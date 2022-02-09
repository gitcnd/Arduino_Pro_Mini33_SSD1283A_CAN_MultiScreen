/*

  This code works on any 5v arduino (The MCP2515 CAN board is unstable at 3.3v) and drives upto 4 LCDs

  For support/issues/etc go to https://github.com/gitcnd/Arduino_Pro_Mini33_SSD1283A_CAN_MultiScreen


  Use this CAN Bus library: https://github.com/coryjfowler/MCP_CAN_lib
    => to install: cd Arduino/libraries; git clone https://github.com/coryjfowler/MCP_CAN_lib
  To drive these CAN_Bus boards: https://www.aliexpress.com/item/33041533951.html

  
  Use this LCD driver: https://github.com/ZinggJM/SSD1283A
    => to install: cd Arduino/libraries; git clone https://github.com/ZinggJM/SSD1283A
  To drive these scereens:  https://www.aliexpress.com/item/1005002378136214.html


  It reads incoming data from CAN_Bus, serial, analogue-inputs, and/or digital pins.

  It outputs the data onto 4 LCD screens.

  Except for the CS wires, all the LCDs are joined toghter, 

  Wiring - LCD <=> ARDUINO
           LED  => Pin9  (PWM brightness)
           SCK  => PIN13 (SCK)
           SDA  => PIN11 (MOSI)
           A0   => PIN8
           RST  => PIN7
           CS   => Pins 5, 6, 4, and 3  (each screen has own pin)

           CAN_Bus <=> ARDUINO
           SO   => PIN12 (MISO)
           SI   => PIN11 (MOSI)
           SCK  => PIN13 (SCK)
           CS   => PIN10 (SS)
           INT  => PIN2
           

  This also requires the custom LCDWIKI_GUI library (included in this distro);
   => to install: cd Arduino/libraries; git clone **TBD** (included)

  And the SerialID library:
   => to install: cd Arduino/libraries; git clone https://github.com/gitcnd/SerialID.git 

  Inspired by https://github.com/lcdwiki/LCDWIKI_SPI/tree/master/Example/Example_03_display_string/display_string
  
*/

#include <SerialID.h>	// This lib is a convenient way to report what code and version is running inside our MCUs (Serial.prints below at boot)
SerialIDset("\n#\tv3.26 " __FILE__ "\t" __DATE__ " " __TIME__); // cd Arduino/libraries; git clone https://github.com/gitcnd/SerialID.git


#include <mcp_can.h>  // Driver for the Chinese CAN_Bus boards; // cd Arduino/libraries; git clone https://github.com/coryjfowler/MCP_CAN_lib
#include <SPI.h>


// CAN0 INT and CS
#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10


// Wire your 4 screens with CD(A0)=8, SDA=13, SCK=11, RST=7, LED=6, and CS= the below:-
#define CS_SCREEN1 4 // 1st LCD
#define CS_SCREEN2 6 // 2nd LCD
#define CS_SCREEN3 5 // 3rd LCD
#define CS_SCREEN4 3 // 4th LCD

#define LCD_CD_PIN_A0 8   // This pin controls whether data or control signals are being sent to the screen (bizarre non-SPI idea...)
#define LCD_RST_PIN 7
#define LCD_BACKLIGHT 9
// #define USE_BACKLIGHT_PWM 1 // Comment this out to just turn the screens on only (no dimming) - it looks like PWM might interferes with CS pin 9/6 ...

#include <LCDWIKI_GUI.h> // *Custom* Core graphics library (has car fonts in it)
#include <SSD1283A.h>    // Hardware-specific library. // cd Arduino/libraries; git clone https://github.com/ZinggJM/SSD1283A

#if defined(__AVR)
// Create an array of screen objects
SSD1283A_GUI scrn[]={ SSD1283A_GUI( CS_SCREEN1, LCD_CD_PIN_A0, LCD_RST_PIN, LCD_BACKLIGHT), 
                      SSD1283A_GUI( CS_SCREEN2, LCD_CD_PIN_A0, LCD_RST_PIN, LCD_BACKLIGHT), 
                      SSD1283A_GUI( CS_SCREEN3, LCD_CD_PIN_A0, LCD_RST_PIN, LCD_BACKLIGHT), 
                      SSD1283A_GUI( CS_SCREEN4, LCD_CD_PIN_A0, LCD_RST_PIN, LCD_BACKLIGHT) };
 //SSD1283A_GUI mylcd1(/*CS=10*/ CS_SCREEN1, /*DC=*/ 8, /*RST=*/ 7, /*LED=*/ 6); //hardware spi,cs,cd,reset,led
#endif


#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define ORANGE  0xFF80

#define downshift2 7 // move all screen-2 things down 7 pixels

// CAN TX Variables
unsigned long prevTX = 0;                                         // Variable to store last execution time
const unsigned int invlTX = 1000;                                 // One second interval constant
byte data[] = {0xAA, 0x55, 0x01, 0x10, 0xFF, 0x12, 0x34, 0x56};   // Generic CAN test data to send
uint8_t can_ok = 0;                                               // Gets set to 1 if the CAN initialized OK

// CAN RX Variables
long unsigned int rxId;
unsigned char len;
//unsigned char rxBuf[8];

#define BufSz 8
union Data {
  uint8_t  b[BufSz/sizeof(uint8_t)];   // 8 bytes
  int8_t  sb[BufSz/sizeof(uint8_t)];   // 8 bytes
  uint16_t i[BufSz/sizeof(uint16_t)];  // 4
  int16_t si[BufSz/sizeof(int16_t)];   // 4
  uint32_t l[BufSz/sizeof(uint32_t)];  // 2
  int32_t sl[BufSz/sizeof(int32_t)];   // 2
  float    f[BufSz/sizeof(float)];
} rxBuf;

// Serial Output String Buffer
char msgString[128];
char fmtString[20]; // temp place converting floats etc to text.

uint8_t last_err=0;
uint8_t last_dve=0;
float last_amps=0.0;
float last_Fkwatts=0.0;
int last_kwatts=0;
int last_soc=0;
long unsigned int last_diag_e=0;
long unsigned int last_diag_w=0;

unsigned char bright=0;
unsigned char degsym[]={246,0}; // 246 is the font dergees-symbol
uint8_t last_temp_i=0;
uint8_t last_temp_m=0;

int X=0; // on-screen CAN diag output
int Y=0;




// Function to pick one or more screens to write to (input is a bit string - e.g. (1 << screen_number)
void sel_screen(int n) {
  if(n&1) digitalWrite(CS_SCREEN1,0); else digitalWrite(CS_SCREEN1,1);
  if(n&2) digitalWrite(CS_SCREEN2,0); else digitalWrite(CS_SCREEN2,1);
  if(n&4) digitalWrite(CS_SCREEN3,0); else digitalWrite(CS_SCREEN3,1);
  if(n&8) digitalWrite(CS_SCREEN4,0); else digitalWrite(CS_SCREEN4,1);
} // sel_screen


// Clear all guage-test readings as soon as we get any real good data
void good_can() {
  if(can_ok<3) { 
    can_ok=3;
    // sel_screen(1+2+4+8); scrn[0].Fill_Screen(BLACK);
    instrument_check(false);	// Replace 888's with ---- everywhere
    signal(2,2);		// Clear "No Valid CAN Signal" message
  }
}


// Convert a Li-iON cell voltage to a charge percentage
int socv(float val) { 
  int pct;
  if(val<3.7) {
    val = 133.33*(val*val*val) - 1365.0*(val*val) + 4671.2*val - 5341.6; // See https://www.powerstream.com/lithium-ion-charge-voltage.htm
  } else {
    val = 175.33*val*val*val - 2304.0*val*val + 10164*val - 14939;
  }
  pct=val;
  if(pct>150)pct=150; // prevent calc problems going nuts; limit to 150% on errors
  if(pct<0)pct=-99; // prevent calc problems going nuts
  return pct;
} // socv



// Function to display (P)ark, (N)eutral, or (D)rive on a display, and optionally the check-engine symbol
void pnd(uint8_t screen_number, uint8_t dve, bool good_can) { // dve is 0 for Park, 1 for Neutral, 2 for Drive
  sel_screen(1 << screen_number);
  pnd2(screen_number, last_dve, 0, good_can);   // un-draw old
  pnd2(screen_number, dve, 1, good_can);        // draw new
  last_dve=dve;                       // Remember what we just drew, so we can un-draw it later
} // pnd

void pnd2(uint8_t s,uint8_t dve, uint8_t draw, bool good_can) { // Draw (and un-draw) for the pnd() Function
  int x,y;
  char pnd[]={'R','N','D'};
  #define PND_SIZE 4
  
  // Draw R, N, or D to show the transmission state:-
  for(uint8_t c=0,x=26,y=1+(2*PND_SIZE+2);c<3;c++) {
    scrn[s].Set_Text_Back_colour(BLACK);
    if(c==dve) {scrn[s].Set_Text_Size( 1+PND_SIZE ); y=y-(2*PND_SIZE+2);} // Make the active letter bigger
    else scrn[s].Set_Text_Size( PND_SIZE );
    
    if(!draw) scrn[s].Set_Text_colour(BLACK);
    else if(c==dve)  scrn[s].Set_Text_colour(0,192,192);
    else      scrn[s].Set_Text_colour(WHITE);

    scrn[s].Set_Text_Cousur(x,y);
    scrn[s].writec(pnd[c]); // Print_String("", 45, 49);
    
    if(c==dve) {y=y+(2*PND_SIZE+2); x+=6+PND_SIZE*6;}
    else x+=PND_SIZE*6; 
  }
} // pnd2



// Function to show (or remove) the Check Engine symbol
void check_engine(uint8_t s, uint8_t err) { //s is screen_number, err=0 means no error (un-draw the logo) err>0 means error
  uint8_t sz=2; // Set the size of the symbol
  sel_screen(1 << s);

  if(err)  scrn[s].Set_Text_colour(RED);
  else     scrn[s].Set_Text_colour(BLACK);
  scrn[s].Set_Text_Back_colour(BLACK);
  scrn[s].Set_Text_Size(sz);
  for(uint8_t c=128,x=45,y=79;c<132;c++,x+=5*sz) { // These are 8 custom font characters that make up a check-engine symbol (4 x 2 characters of 6wide x 8high pixels each)
    scrn[s].Set_Text_Cousur(x,y);
    scrn[s].writec(c); // Print_String("", 45, 49);
    //scrn[s].Print_String("", 45, 49+8);
  }
  for(uint8_t c=132,x=45,y=79+8*sz;c<136;c++,x+=5*sz) { // The next 4 from the above 4 custom font chars
    scrn[s].Set_Text_Cousur(x,y);
    scrn[s].writec(c); // Print_String("", 45, 49);
    //scrn[s].Print_String("", 45, 49+8);
  }
} // check_engine




// Function to display the current AMPS being drawn
void amps(uint8_t screen_number, float val, bool good_can) { 
  if(int(last_amps*10) != int(val*10)) { // Skip re-drawing anything that hasn't changed
    sel_screen(1 << screen_number);
    amps2(screen_number, last_amps, 0, good_can);	// un-draw old
    amps2(screen_number, val, 1, good_can);		// draw new
    last_amps=val;                        		// Remember what we just drew, so we can un-draw it later
  }
} // amps

void amps2(uint8_t s,float val, uint8_t draw, bool good_can) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else scrn[s].Set_Text_colour(0,192,192); // teal
  if(val<=-100) {
    dtostrf(val, 6, 0, fmtString); 	// -888.5	// 6 is the length. negative means left-align. 1 is the decimal places
  } else {
    dtostrf(val, 5, 1, fmtString); 	// -888.5	// 6 is the length. negative means left-align. 1 is the decimal places
  }
  if(good_can) 
    sprintf(msgString, "%s", &fmtString);
  else
    sprintf(msgString, "%s", "---.-");

  #define AMP_SIZE 4  // 4 * 6 = 24px wide, 4 * 8 = 32px high
  scrn[s].Set_Text_Size( AMP_SIZE );  
  //scrn[s].Print_String(msgString, 130/2 - (3 * 6 * AMP_SIZE)/2 , 1, 0, ' ',10); // "center" for 3 digits   ( num,  x, y, length, filler, base)
  scrn[s].Print_String(msgString, 0 , 1 + downshift2); //

  if(draw) {
    scrn[s].Set_Text_colour(WHITE);
    scrn[s].Set_Text_Size( AMP_SIZE-1 );  
    scrn[s].Print_String("AMPS", 130/2 - (3 * 6 * AMP_SIZE)/2, 8*AMP_SIZE + downshift2);
  }
} // amps2



// Function to display watts being drawn (or negative - regen - added)
void Fkwatts(uint8_t screen_number, float val, bool good_can) { 
  if( last_Fkwatts != val ) { // Skip re-drawing anything that hasn't changed
    sel_screen(1 << screen_number);
    Fkwatts2(screen_number, last_Fkwatts, 0, good_can);   // un-draw old
    Fkwatts2(screen_number, val, 1, good_can);         // draw new
    last_Fkwatts=val;                        // Remember what we just drew, so we can un-draw it later
  }
} // Fkwatts

void Fkwatts2(uint8_t s,float val, uint8_t draw, bool good_can) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else if(val>=0) scrn[s].Set_Text_colour(0,192,192); // teal
  else  scrn[s].Set_Text_colour(GREEN); // make it green for regen :-)

  #define KW_SIZE 4  // 4 * 6 = 24px wide, 4 * 8 = 32px high
  scrn[s].Set_Text_Size( KW_SIZE );  
  if((val<1000.0)&&(val>-100.0)) {
    dtostrf(val, 5, 1, fmtString); 	// 432.1 -99.1	// 5 is the length. negative means left-align. 1 is the decimal places
  } else {
    dtostrf(val, 5, 0, fmtString); 	// 23000 -4320	// 5 is the length. negative means left-align. 1 is the decimal places
  }

  //scrn[s].Print_String(fmtString, 130/2 - (3 * 6 * KW_SIZE)/2 - KW_SIZE*3 , 66 + downshift2);
  scrn[s].Print_String(fmtString, 10 , 66 + downshift2);

  if(draw) { // No need to un-draw the static text
    scrn[s].Set_Text_colour(WHITE);
    scrn[s].Set_Text_Size( KW_SIZE-1 );  
    scrn[s].Print_String(" kW", 130/2 - (3 * 6 * KW_SIZE)/2 , 66+8*KW_SIZE + downshift2);
    //scrn[s].Print_String("watts", 130/2 - (5 * 6 * KW_SIZE)/2 , 66+8*KW_SIZE + downshift2);
  }

} // Fkwatts2





// Function to display watts being drawn (or negative - regen - added)
void kwatts(uint8_t screen_number, int val, bool good_can) { 
  if( last_kwatts != val ) { // Skip re-drawing anything that hasn't changed
    sel_screen(1 << screen_number);
    kwatts2(screen_number, last_kwatts, 0, good_can);   // un-draw old
    kwatts2(screen_number, val, 1, good_can);         // draw new
    last_kwatts=val;                        // Remember what we just drew, so we can un-draw it later
  }
} // kwatts

void kwatts2(uint8_t s,int val, uint8_t draw, bool good_can) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else if(val>=0) scrn[s].Set_Text_colour(0,192,192); // teal
  else  scrn[s].Set_Text_colour(GREEN); // make it green for regen :-)

  #define KW_SIZE 4  // 4 * 6 = 24px wide, 4 * 8 = 32px high
  scrn[s].Set_Text_Size( KW_SIZE );  
//  if(good_can)
    scrn[s].Print_Number_Int(val, 130/2 - (3 * 6 * KW_SIZE)/2 - KW_SIZE*3 , 66 + downshift2, 3, ' ',10); // "center" for 2 digits   ( num,  x, y, length, filler, base)
//  else
//    scrn[s].Print_String(" --", 130/2 - (3 * 6 * KW_SIZE)/2 - KW_SIZE*3 , 66 + downshift2);

  if(draw) { // No need to un-draw the static text
    scrn[s].Set_Text_colour(WHITE);
    scrn[s].Set_Text_Size( KW_SIZE-1 );  
    scrn[s].Print_String(" kW", 130/2 - (3 * 6 * KW_SIZE)/2 , 66+8*KW_SIZE + downshift2);
    //scrn[s].Print_String("watts", 130/2 - (5 * 6 * KW_SIZE)/2 , 66+8*KW_SIZE + downshift2);
  }



} // kwatts2



// Function to display the remaining battery capacity in numbers (%) 
void soc(uint8_t screen_number, int val, bool good_can) { 
  if( last_soc != val ) { // Skip re-drawing anything that hasn't changed
    sel_screen(1 << screen_number);
    soc2(screen_number, last_soc, 0,good_can);	// un-draw old
    soc2(screen_number, val, 1,good_can);	// draw new
    last_soc=val;				// Remember what we just drew, so we can un-draw it later
  }
} // kwatts

void soc2(uint8_t s,int val, uint8_t draw, bool good_can) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else scrn[s].Set_Text_colour(0,192,192); // teal

  #define SOC_SIZE 4  // 4 * 6 = 24px wide, 4 * 8 = 32px high
  scrn[s].Set_Text_Size( SOC_SIZE-1 );  
  scrn[s].Print_String("%",  130/2 - (3 * 6 * SOC_SIZE)/2 + SOC_SIZE*3 + (2 * 6 * SOC_SIZE)   , 66+8); // Do this first, so big numbers overwrite the % instead of other way around

  scrn[s].Set_Text_Size( SOC_SIZE );
  if(good_can)
    scrn[s].Print_Number_Int(val, 130/2 - (3 * 6 * SOC_SIZE)/2 + SOC_SIZE*3 , 66, 3, ' ',10); // "center" for 2 digits   ( num,  x, y, length, filler, base)
  else
    scrn[s].Print_String(" --", 130/2 - (3 * 6 * SOC_SIZE)/2 + SOC_SIZE*3 , 66);

  if(draw) {
    scrn[s].Set_Text_colour(WHITE);
    scrn[s].Set_Text_Size( SOC_SIZE-1 );  
    scrn[s].Print_String("SoC", 130/2 - (3 * 6 * SOC_SIZE)/2 + (SOC_SIZE*6/2) , 66+8*SOC_SIZE);
  }
} // soc2



// Function to display pretty temperature data
void tempC(uint8_t screen_number, int val, int last_val, int x_offset, bool good_can) { 
  sel_screen(1 << screen_number);
  tempC2(screen_number, last_val, 0, x_offset, good_can);   // un-draw old
  tempC2(screen_number, val, 1, x_offset, good_can);         // draw new
} // tempC

void tempC2(uint8_t s,int val, uint8_t draw, int x_offset, bool good_can) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else scrn[s].Set_Text_colour(0,192,192); // teal

  #define TEMP_SIZE 2  // 4 * 6 = 24px wide, 4 * 8 = 32px high
  scrn[s].Set_Text_Size( TEMP_SIZE );  
  
  if(good_can)
    sprintf(msgString, "%d%sC", val, degsym); // degrees symbol: (246 I think)  ö
  else
    sprintf(msgString, "--%sC", degsym);
 
  scrn[s].Print_String(msgString, x_offset + 10, 130 - 8*TEMP_SIZE-10); // Do it all in teal, so the degrees-C is in the right spot
  if(draw) { // re-draw just the number, in white now
    scrn[s].Set_Text_colour(WHITE);
    sprintf(msgString, "%d", val);
    if(good_can)
      scrn[s].Print_String(msgString, x_offset + 10, 130 - 8*TEMP_SIZE-10); // Do it all in teal, so the degrees-C is in the right spot
    else
      scrn[s].Print_String("--",      x_offset + 10, 130 - 8*TEMP_SIZE-10);
  }
  // Draw a thermometer in graphics here... ?
} // tempC2



// Function to display 32bit Diagnostic bit string
void diag(uint8_t s,unsigned long int val, uint8_t draw, int x_offset, int y_offset, char *label, uint8_t colr) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else { 
    if(colr==1) scrn[s].Set_Text_colour(RED);
    else if(colr==2) scrn[s].Set_Text_colour(ORANGE);
    else if(colr==3) scrn[s].Set_Text_colour(YELLOW);
    else if(colr>3) scrn[s].Set_Text_colour(WHITE);
  }
  scrn[s].Set_Text_Size( 1 );
  sprintf(msgString, "%s%08lX", label, val);
  //  scrn[s].Print_Number_Int(val, 0, 16, 0, ' ',16);
  scrn[s].Print_String(msgString, x_offset,         y_offset);
} // diag



// Function to display 16bit Diagnostic bit string
void diags(uint8_t s,unsigned int val, uint8_t draw, int x_offset, int y_offset, char *label, uint8_t colr) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else { 
    if(colr==1) scrn[s].Set_Text_colour(RED);
    else if(colr==2) scrn[s].Set_Text_colour(ORANGE);
    else if(colr==3) scrn[s].Set_Text_colour(YELLOW);
    else if(colr>3) scrn[s].Set_Text_colour(WHITE);
  }
  scrn[s].Set_Text_Size( 1 );
  sprintf(msgString, "%s%04X", label, val);
  //  scrn[s].Print_Number_Int(val, 0, 16, 0, ' ',16);
  scrn[s].Print_String(msgString, x_offset,         y_offset);
} // diags



// Function to display voltage
void volt_v(uint8_t s, float val, uint8_t draw, int x_offset, int y_offset, char *label, uint8_t nlen, uint8_t digs) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else scrn[s].Set_Text_colour(WHITE);
  dtostrf(val, nlen, digs, &msgString[100]);
  sprintf(msgString, label, &msgString[100]);
  scrn[s].Set_Text_Size( 1 );
  scrn[s].Print_String(msgString, x_offset, y_offset);
} // volt_v



// Function to display percentage
void percent(uint8_t s, int val, uint8_t draw, int x_offset, int y_offset, char *label) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else scrn[s].Set_Text_colour(WHITE);
  sprintf(msgString, label, val);
  scrn[s].Set_Text_Size( 1 );
  scrn[s].Print_String(msgString, x_offset, y_offset);
} // percent



// Function to display a message about CAN Bus Data
void signal(uint8_t s,uint8_t has_sig) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Size(1);
  if(has_sig==1) {//	  1234567890123 = 13 = 78px wide
    scrn[s].Set_Text_Back_colour(BLACK);
    scrn[s].Set_Text_colour(RED);
    //scrn[s].Print_String("               ", 20, 57);
    scrn[s].Print_String(" No Valid CAN Signal ", 2, 57);
  } else if(has_sig==2) { // All OK now
    scrn[s].Set_Text_Back_colour(BLACK);
    scrn[s].Set_Text_colour(RED);
    //scrn[s].Print_String("               ", 20, 57);
    scrn[s].Print_String("                     ", 2, 57);
  } else {
    scrn[s].Set_Text_Back_colour(RED);
    scrn[s].Set_Text_colour(WHITE);
    scrn[s].Print_String(" No CAN Signal ", 20, 57);
  }
} // signal



// Connector functions between incoming CAN data and screen display functions

void diag_e(uint8_t screen_number,  unsigned long int val) { if( last_diag_e != val ) { diag(screen_number,  last_diag_e,  0, 0,    13*8+3,  (char*)"E:",1);   last_diag_e=val;  diag(screen_number,  val, 1, 0,    13*8+3, (char*)"E:",1); }}
void diag_w(uint8_t screen_number,  unsigned long int val) { if( last_diag_w != val ) { diag(screen_number,  last_diag_w,  0, 0,    14*8+3,  (char*)"W:",2);   last_diag_w=val;  diag(screen_number,  val, 1, 0,    14*8+3,  (char*)"W:",2); }}

void temp_i(uint8_t screen_number,  uint8_t val, bool good_can) { if( last_temp_i != val ) { tempC(screen_number,  val, last_temp_i,  0, good_can);       last_temp_i=val;  }}
void temp_m(uint8_t screen_number,  uint8_t val, bool good_can) { if( last_temp_m != val ) { tempC(screen_number,  val, last_temp_m,  130/2, good_can);   last_temp_m=val;  }}

//void diag_n(uint8_t screen_number,  unsigned long int val) { if( last_diag_n != val ) { diag(screen_number,  last_diag_n,  0, 0,    15*8+3, "N:",3);   last_diag_n=val;  diag(screen_number,  val, 1, 0,    15*8+3, "N:",3); }}
//void diags_r(uint8_t screen_number,      unsigned int val) { if( last_diags_r != val ) { diags(screen_number, last_diags_r, 0, 11*6, 14*8+3, "rsv:",4); last_diags_r=val; diags(screen_number, val, 1, 11*6, 14*8+3, "rsv:",4); }}
//void diags_i(uint8_t screen_number,      unsigned int val) { if( last_diags_i != val ) { diags(screen_number, last_diags_i, 0, 11*6, 15*8+3, "Int:",5); last_diags_i=val; diags(screen_number, val, 1, 11*6, 15*8+3, "Int:",5); }}

//void volt_i(uint8_t screen_number, float val) { if( last_volt_i != val ) { volt_v(screen_number, last_volt_i, 0, 3*6, 1+10 * 8, "%s", 5, 1); last_volt_i=val;  volt_v(screen_number, val, 1, 0, 1+10 * 8, "vIN%s", 5, 1); }} 
//void volt_e(uint8_t screen_number, float val) { if( last_volt_e != val ) { volt_v(screen_number, last_volt_e, 0, 3*6, 1+11 * 8, "%s", 7, 3); last_volt_e=val;  volt_v(screen_number, val, 1, 0, 1+11 * 8, "vEX%s", 5, 1); }} 
//void volt_b(uint8_t screen_number, float val) { if( last_volt_b != val ) { volt_v(screen_number, last_volt_b, 0, 3*6, 1+12 * 8, "%s", 5, 1); last_volt_b=val;  volt_v(screen_number, val, 1, 0, 1+12 * 8, "BUS%s", 5, 1); }} 

//void pc_t(uint8_t screen_number, int val) { if( last_pc_t != val ) { percent(screen_number, last_pc_t, 0, (11+3)*6, 1+11 * 8, "%d"); last_pc_t=val; percent(screen_number, val, 1, (11)*6, 1+11 * 8, "Th %d %%"); }}
//void pc_m(uint8_t screen_number, int val) { if( last_pc_m != val ) { percent(screen_number, last_pc_m, 0, (11+3)*6, 1+12 * 8, "%d"); last_pc_m=val; percent(screen_number, val, 1, (11)*6, 1+12 * 8, "Mt %d %%"); }}
//void pc_c(uint8_t screen_number, int val) { if( last_pc_c != val ) { percent(screen_number, last_pc_c, 0, (11+3)*6, 1+13 * 8, "%d"); last_pc_c=val; percent(screen_number, val, 1, (11)*6, 1+13 * 8, "Cp %d %%"); }}



// Light up all the instruments like analogue cars do, to check all guages are working...
void instrument_check(bool good_can) { // good_can=true (for instrument_check 888's) or false (to replace 888's with ----)
  // good_can    888-check-888    ; else    ---_wait_---
  if(good_can)   check_engine(0,1) ; else check_engine(0,0);
  if(good_can)   pnd(0,1,good_can); else  pnd(0,1,good_can);
  if(good_can)   amps(2,888.8,good_can); else  amps(2,987.6,good_can);
  //if(good_can)   kwatts(2,-88,good_can); else  kwatts(2,-98,good_can);
  if(good_can)   Fkwatts(2,-88.8,good_can); else  Fkwatts(2,-98.8,good_can);
  if(good_can)   soc(3,88,good_can); else soc(3,123,good_can);	// Note that the value must be different, otherwise the screen-print code gets skipped.
  //show_font(1);  
  //if(good_can)   tempC(1,88,0,0,good_can); else    tempC(1,123,0,0,good_can);
  //if(good_can)   tempC(1,88,0,130/2,good_can); else  tempC(1,123,0,130/2,good_can);
  if(good_can)   temp_i(1,88,good_can); else  temp_i(1,123,good_can);
  if(good_can)   temp_m(1,88,good_can); else  temp_m(1,123,good_can);
  signal(2,0);	// Show "No CAN Signal"
} // instrument_check


/* Debug subs


void show_font(uint8_t s) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Back_colour(BLACK);
  scrn[s].Set_Text_colour(WHITE);
  uint8_t c=0;
  for(int y=0;y<128-1;y+=8) {
    for(int x=0;x<126-1;x+=6) {
      scrn[s].Set_Text_Cousur(x,y); // I did not write the lib - whoever did probably doesn't speak english as their first language :-)
      scrn[s].writec(c++); 
    }    
  }
} // show_font


void demo_screen() {
  for(int s=0; s<4; s++) {
    int sbit=1<<s;
    sel_screen(sbit);
    // scrn[s].setRotation(r); // If you get an error in this line, you've forgotten to pick an Arduino board to compile to ( Tools => Board => Arduin AVR Boards => Arduino Nano )
  
    scrn[s].Set_Text_Mode(0);
  
    scrn[s].Fill_Screen(0x0000);
    scrn[s].Set_Text_colour(RED);
    scrn[s].Set_Text_Back_colour(BLACK);
    scrn[s].Set_Text_Size(1);
    scrn[s].Print_String("Hello World!", 0, 0);
    scrn[s].Print_Number_Float(01234.56789, 2, 0, 8, '.', 0, ' ');  
    scrn[s].Print_Number_Int(0xDEADBEF, 0, 16, 0, ' ',16);

    scrn[s].Set_Text_colour(GREEN);
    scrn[s].Set_Text_Size(2);
    scrn[s].Print_String("Screen", 0, 32);
    scrn[s].Print_Number_Int(s, 100, 32, 0, ' ',32);
    scrn[s].Print_Number_Float(01234.56789, 2, 0, 48, '.', 0, ' ');  
    scrn[s].Print_Number_Int(0xDEADBEF, 0, 64, 0, ' ',16);

    scrn[s].Set_Text_colour(BLUE);
    scrn[s].Set_Text_Size(3);
    scrn[s].Print_String("Hello", 0, 86);
    scrn[s].Print_Number_Float(01234.56789, 2, 0, 110, '.', 0, ' ');  
    //scrn[s].Print_Number_Int(0xDEADBEF, 0, 134, 0, ' ',16);

    pnd(s, s%3); // 0 (P) ,1 (N), ,2 (D)

  } // s

#ifdef USE_BACKLIGHT_PWM
  if(0) { // example of changing screen brightness
    analogWrite(LCD_BACKLIGHT,bright); // This is how you change the LCD brightness from no backlight (0) to full on (255)
    bright+=32;
    if(bright==128)bright--; // so we hit 255 later
    if(bright<32)bright=0;   // so we hit 0 as well
    // $c=0;while(1){$c=0 if($c<32); print "$c "; $c+=32; $c-- if($c==128); $c-=256 if($c>255);}
  }
#endif
} // demo_screen

*/



/******************************************************************************
**									     **
**				     setup()				     **
**									     **
******************************************************************************/

void setup() {
#ifdef USE_BACKLIGHT_PWM
  analogWrite(LCD_BACKLIGHT,0); // Turn off the backlight at startup (also inits this pin)
#else
  pinMode(LCD_BACKLIGHT,OUTPUT);
  digitalWrite(LCD_BACKLIGHT,1); // Turn on the backlight
#endif
  SerialIDshow(115200);         // starts Serial, and outputs our version and build info to USB if connected

  //other CAN boards might need: if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) // Our boards have 8mhz xtal... and we are using 250kbps

  // Initialize MCP2515 running at 8MHz with a baudrate of 250kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
    can_ok=1;
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515..."); // can_ok already=0
  }
  
  // Set NORMAL mode, since we would be in loopback mode by default.
  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT);                           // Configuring pin for /INT input
  
  pinMode(CS_SCREEN1,OUTPUT);
  pinMode(CS_SCREEN2,OUTPUT);
  pinMode(CS_SCREEN3,OUTPUT);
  pinMode(CS_SCREEN4,OUTPUT);
  sel_screen(0); // de-select all
  
  // MUST Init all at the same time
  sel_screen(1+2+4+8);        // select all
  scrn[0].init();             // This writes to all screens at once
  scrn[0].Fill_Screen(BLACK); // so does this
  if(!can_ok) {
    scrn[0].Set_Text_colour(WHITE);
    scrn[0].Set_Text_Back_colour(BLACK);
    scrn[0].Set_Text_Size(2);
    scrn[0].Print_String("CAN Init", 17, 55);
    scrn[0].Print_String( "failed",  29, 65);
    check_engine(0,1);
  } 
  //else { // testing
    //scrn[0].Set_Text_colour(WHITE);
    //scrn[0].Set_Text_Back_colour(BLACK);
    //scrn[0].Set_Text_Size(2);
    //scrn[0].Print_String("Tesyota", 23, 55);
    //check_engine(0,1);
  //}
  sel_screen(0); // de-select them
  delay(10);
  for(int s=0; s<4; s++) {
    sel_screen(1<<s);
    scrn[s].setRotation(2);     // Orient all the screens so "UP is UP"
  }

#ifdef USE_BACKLIGHT_PWM
  for(int i=1;i<256;i++) {
    analogWrite(LCD_BACKLIGHT,i); // Turn up the screen brightness gradually (kinder on the power supply)
    delay(5);
  }
#endif
  
  instrument_check(true); // Show 888 on everything (fake a "good_can" to force all the 888's)
  delay(5000);
  
} // setup



/******************************************************************************
**									     **
**				     loop()				     **
**									     **
******************************************************************************/
void loop() 
{
  char buf[50]; // to assemble a big message
  char one[2];  // to output it one-byte-a-a-time

#ifdef USE_BACKLIGHT_PWM
  //analogWrite(LCD_BACKLIGHT,bright); // This is how you change the LCD brightness from no backlight (0) to full on (255)
#endif

  for(int ii=0;ii<3;ii++) {


  if(!digitalRead(CAN0_INT))  {                        // If CAN0_INT pin is low, read receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf.b);              // Read data: len = data length, buf = data byte(s)

    if ((rxId>0)||(len>0)) { // we were getting spurious zeros too much...
      if(can_ok<2) { can_ok=2; signal(2,1); }		// "No Valid CAN Signal"


      if((rxId & 0x80000000) == 0x80000000) {           // Determine if ID is standard (11 bits) or extended (29 bits)
        unsigned long hbcid=rxId & 0x1FFFFFFF; 		// Process extended CAN ID's (if any) here.
	//if(hbcid==0x22f015) { // PID: 22f015, OBD Header: 7E3, Equation: ((((A*256)+B)-32767.0)/10.0)*-1
	//  good_can();
	//  //long can_amps=rxBuf[0]<<8+rxBuf[1]-32767;
	//  long can_amps=rxBuf.b[0]; can_amps+=rxBuf.b[1]; can_amps-=32767;
	//  float can_ampsf=float(can_amps); can_ampsf=can_ampsf/10.0 * -1.0;
	//  amps(2,can_ampsf,true);
	//}
      } else {
	if(rxId==0x501) { // Ken custom CANBUS defined message for AMPS and VOLTS
	  good_can();
	  // amps(2,rxBuf.si[0],true);		// Use this if AMPS is a signed whole integer
	  float can_ampsf=float(rxBuf.si[0]);  can_ampsf=can_ampsf/10.0 ;
	  amps(2,can_ampsf,true);
	  float can_volts=float(rxBuf.si[1]);  can_volts=can_volts/10.0 ;
	  //long lwatts=rxBuf.si[0];
          //lwatts=lwatts*rxBuf.si[1]; // watts = volts * amps  (nb volts is *10 too - eg 1462 instead of 146.2)
          //lwatts=lwatts/100000;      // /10 for the amp scale, /10 for the volt scale, /1000 to convert to kW
          //int iwatts=lwatts;
          //kwatts(2,iwatts,true);
          Fkwatts(2,can_ampsf * can_volts ,true);

          // kwatts(2,rxBuf.si[1],true); // 1462

        //} else if(rxId==0x7E3 || rxId==0x015) { // PID: 22f015, OBD Header: 7E3, Equation: ((((A*256)+B)-32767.0)/10.0)*-1
	//  good_can();
	//  //long can_amps=rxBuf[0]<<8+rxBuf[1]-32767;
	//  //float can_ampsf=can_amps; can_ampsf=can_ampsf/10.0 * -1.0;
	//  long can_amps=rxBuf.b[0]; can_amps+=rxBuf.b[1]; can_amps-=32767;
	//  float can_ampsf=float(can_amps); can_ampsf=can_ampsf/10.0 * -1.0;
	//  amps(2,can_ampsf,true);
	} else if(rxId==0x401) {			// PID: b48401, OBD Header: 7E3, Equation: H - state of charge etc
	  good_can();					// Standard ID: 0x401       DLC: 8  Data: 0x47 0x47 0x00 0x3D 0x02 0xFF 0xFF 0x55
  	  soc(3,rxBuf.b[7],true);			// Standard ID: 0x401       DLC: 8  Data: 0x47 0x47 0x00 0x00 0x00 0x00 0x00 0x55
	  temp_i(1,rxBuf.b[0]-40,true);			// Controller Temp........................^^^^
	  temp_m(1,rxBuf.b[1]-40,true);			// Motor Temp..................................^^^^
	  //rxBuf[2]					// Fault Code.......................................^^^^
	  //rxBuf[3]<<8+rxBuf[4]			// Motor RPM.............................................^^^^^^^^
	  //rxBuf[5]<<8+rxBuf[6]			// Motor Torque -100:100 ..........................................^^^^^^^^
	  
	} else if(rxId==0x701) {			// Standard ID: 0x701       DLC: 1  Data: 0x05 
	  good_can();
	} else if(rxId==0x181) {			// Standard ID: 0x181       DLC: 8  Data: 0x99 0x01 0x00 0x20 0x6D 0x00 0x00 0x39
	  good_can();
	  //uint64_t e=rxBuf[3]<<24 + rxBuf[2]<<16 + rxBuf[1]<<8 + rxBuf[0];
	  //uint64_t w=rxBuf[7]<<24 + rxBuf[6]<<16 + rxBuf[5]<<8 + rxBuf[4];
	  diag_e(0,rxBuf.l[0]); // e;
	  diag_w(0,rxBuf.l[1]); // w
	  // Also, for the P, N, D the ID for this is 0x181 byte 7
	  // N = 18 or 1A    	     11000 
	  //                 	     11010
	  // D = 39 or 3B    	    111001
	  //                 	    111011
	  // R = 59 or 5B    	   1011001
	  //                	   1011011
	  if((rxBuf.b[7] & 	0b01100001) == 0) pnd(0, 1, true);		// N
	  else if((rxBuf.b[7]& 	0b01100001) == 0b00100001) pnd(0, 2, true);	// D
	  else if((rxBuf.b[7]& 	0b01100001) == 0b01000001) pnd(0, 0, true);	// R


	} else if(rxId==0x736) {			// Standard ID: 0x281       DLC: 8  Data: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
	  good_can();
	}
      }
  
/*

 Standard ID: 0x036       DLC: 8  Data: 0x13 0x9C 0xB0 0x00 0x47 0x9C 0xB4 0x34
 Standard ID: 0x036       DLC: 8  Data: 0x01 0x9C 0xC0 0x00 0x47 0x9C 0xC4 0x42
 Standard ID: 0x036       DLC: 8  Data: 0x1A 0x9C 0xB8 0x00 0x47 0x9C 0xBB 0x4A
 Standard ID: 0x036       DLC: 8  Data: 0x0F 0x9C 0xB8 0x00 0x47 0x9C 0xBB 0x3F
 Standard ID: 0x036       DLC: 8  Data: 0x04 0x9C 0x9A 0x00 0x47 0x9C 0x9F 0xFA
 Standard ID: 0x036       DLC: 8  Data: 0x1D 0x9C 0xDA 0x00 0x47 0x9C 0xE0 0x94
 Standard ID: 0x036       DLC: 8  Data: 0x12 0x9C 0xB3 0x00 0x47 0x9C 0xB4 0x36
 Standard ID: 0x701       DLC: 1  Data: 0x05 
 Standard ID: 0x036       DLC: 8  Data: 0x1F 0x9C 0xD6 0x00 0x47 0x9C 0xDA 0x8C
 Standard ID: 0x036       DLC: 8  Data: 0x1A 0x9C 0xB6 0x00 0x47 0x9C 0xBB 0x48
 Standard ID: 0x701       DLC: 1  Data: 0x05 
 Standard ID: 0x281       DLC: 8  Data: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
 Standard ID: 0x701       DLC: 1  Data: 0x05 

Standard ID: 0x181       DLC: 8  Data: 0x99 0x01 0x00 0x20 0x6D 0x00 0x00 0x18
Standard ID: 0x181       DLC: 8  Data: 0x99 0x01 0x00 0x20 0x6D 0x00 0x00 0x39
Standard ID: 0x181       DLC: 8  Data: 0x99 0x03 0x00 0x20 0x6D 0x00 0x00 0x18
Standard ID: 0x181       DLC: 8  Data: 0x99 0x03 0x00 0x20 0x6D 0x00 0x00 0x39
Standard ID: 0x181       DLC: 8  Data: 0xD9 0x03 0x00 0x20 0x6D 0x00 0x00 0x3B
Standard ID: 0x401       DLC: 8  Data: 0x47 0x47 0x00 0x00 0x00 0x00 0x00 0x55
Standard ID: 0x401       DLC: 8  Data: 0x47 0x47 0x00 0x11 0x03 0x0F 0x00 0x55
Standard ID: 0x401       DLC: 8  Data: 0x47 0x47 0x00 0x29 0x01 0xFF 0xFF 0x55
Standard ID: 0x401       DLC: 8  Data: 0x47 0x47 0x00 0x3D 0x02 0xFF 0xFF 0x55
Standard ID: 0x401       DLC: 8  Data: 0x47 0x47 0x00 0x66 0x03 0x00 0x00 0x55
Standard ID: 0x401       DLC: 8  Data: 0x47 0x47 0x00 0x94 0x00 0xFF 0xFF 0x55
Standard ID: 0x401       DLC: 8  Data: 0x48 0x47 0x00 0x00 0x00 0x00 0x00 0x55
Standard ID: 0x401       DLC: 8  Data: 0x48 0x47 0x00 0x3F 0x01 0xFF 0xFF 0x55
Standard ID: 0x6B1       DLC: 8  Data: 0x01 0x90 0x00 0x9F 0x1F 0x1F 0x00 0x27
Standard ID: 0x6B1       DLC: 8  Data: 0x01 0x90 0x00 0xA0 0x1F 0x1F 0x00 0x28
Standard ID: 0x708       DLC: 8  Data: 0x06 0xA7 0xB0 0xB0 0x01 0x90 0xAE 0x00


*/

     // Below - debug: can data output to serial

     if((rxId & 0x80000000) == 0x80000000) {            // Determine if ID is standard (11 bits) or extended (29 bits)
       sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
       sprintf(buf,"EX:%.8lX L:%1d d:", (rxId & 0x1FFFFFFF), len);
     } else {
       sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
       sprintf(buf,"ID:%.3lX L:%d ", rxId, len);
     }
   
     Serial.print(msgString);
     
   
     if((rxId & 0x40000000) == 0x40000000){            // Determine if message is a remote request frame.
       //sprintf(msgString, " REMOTE REQUEST FRAME");
       Serial.print("RMT REQ");
       if(strlen(buf)<46) sprintf(&buf[strlen(buf)],"RMT"); // buf is 50 bytes
     } else {
       for(byte i = 0; i<len; i++){
         sprintf(msgString, " 0x%.2X", rxBuf.b[i]);				// 0        1         2         3         4         5
         Serial.print(msgString);						// 12345678901234567890123456789012345678901234567890
         if(strlen(buf)<46) sprintf(&buf[strlen(buf)],"%.2X ", rxBuf.b[i]);	// EX:12345678 L:123 d:12 34 56 78 90 12 34 56.
       }
     }
 
#ifdef SHOW_CAN_DATA_ON_LCD

     // Send data to LCD
     #define CAN_SCRN 3
     sel_screen(1<<CAN_SCRN);
     scrn[CAN_SCRN].Set_Text_Back_colour(BLACK);
     scrn[CAN_SCRN].Set_Text_colour(YELLOW);
     scrn[CAN_SCRN].Set_Text_Size(1);
     one[1]=0;
     for(int i=0;i<strlen(buf);i++) { // Send data to LCD, wrapping as needed
       //one[0]=" ";    textPrint(0,X,Y,BLACK,BLACK,1,one);     // erase
       //one[0]=buf[i]; textPrint(0,X,Y,YELLOW,BLACK,1,one); // write char
 
       //scrn[CAN_SCRN].Set_Text_colour(BLACK);
       one[0]=" "; scrn[CAN_SCRN].Print_String(one, X, Y);
       one[0]=buf[i]; scrn[CAN_SCRN].Print_String(one, X, Y);
       
       X=X+6;  if(X>=126) { X=0; Y=Y+8; if(Y>=127)Y=0;}  // advance
     }	  
#endif
 
     Serial.println();

   } // nothing
  
  }else {
    //Serial.print("pin "); Serial.print(CAN0_INT); Serial.println(" is high: no data to read");
  } // CAN0_INT


if(0){ //  example of how to send:-
  if(millis() - prevTX >= invlTX){                    // Send this at a one second interval. 
    CAN0.enOneShotTX(); delay(10);
    prevTX = millis();
    byte sndStat = CAN0.sendMsgBuf(0x111, 8, data); data[7]++;
    delay(10);CAN0.disOneShotTX();delay(10);
    if(sndStat == CAN_OK)
      Serial.println("Message Sent Successfully!");
    else
      Serial.println("Error Sending Message...");
  }
} // send test



  }// for //delay(3000);
} // loop
