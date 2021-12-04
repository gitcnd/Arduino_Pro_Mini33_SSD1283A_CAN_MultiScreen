// WORKS!  With 4 screens & Chinese CAN

/*

  This code runs on any 3.3v arduino.  

  Uses this CAN Bus library: https://github.com/coryjfowler/MCP_CAN_lib

  To drive these CAN_Bus boards: https://www.aliexpress.com/item/33041533951.html

  Using these scereens:  https://www.aliexpress.com/item/1005002378136214.html
  With this driver: https://github.com/ZinggJM/SSD1283A
  
  It reads incoming data from CAN_Bus, serial, analogue-inputs, and/or digital pins.

  It outputs the data onto 4 LCD screens.

  Except for the CS wires, all the LCDs are joined toghter, 

  Wiring - LCD <=> ARDUINO
           LED  => Pin6  (PWM brightness)
           SCK  => PIN13 (SCK)
           SDA  => PIN11 (MOSI)
           A0   => PIN8
           RST  => PIN7
           CS   => Pins 5, 9, 4, and 3  (each screen has own pin)

           CAN_Bus <=> ARDUINO
           SO   => PIN12 (MISO)
           SI   => PIN11 (MOSI)
           SCK  => PIN13 (SCK)
           CS   => PIN10 (SS)
           INT  => PIN2
           

*/


// original code taken from https://github.com/lcdwiki/LCDWIKI_SPI/tree/master/Example/Example_03_display_string/display_string


#include <SerialID.h>  // So we know what code and version is running inside our MCUs
SerialIDset("\n#\tv2.2 " __FILE__ "\t" __DATE__ " " __TIME__);


#include <mcp_can.h>
#include <SPI.h>

// CAN TX Variables
unsigned long prevTX = 0;                                        // Variable to store last execution time
const unsigned int invlTX = 1000;                                // One second interval constant
byte data[] = {0xAA, 0x55, 0x01, 0x10, 0xFF, 0x12, 0x34, 0x56};  // Generic CAN data to send

// CAN RX Variables
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];

// Serial Output String Buffer
char msgString[128];

// CAN0 INT and CS
#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10



// Wire your 4 screens with CD(A0)=8, SDA=13, SCK=11, RST=8, LED=6, and CS= the below:-

#define CS_SCREEN1 5
#define CS_SCREEN2 9
#define CS_SCREEN3 4
#define CS_SCREEN4 3


#include <LCDWIKI_GUI.h> //Core graphics library
#include <SSD1283A.h> //Hardware-specific library

#if defined(__AVR)
SSD1283A_GUI scrn[]={ SSD1283A_GUI(CS_SCREEN1,8,7,6), 
                      SSD1283A_GUI(CS_SCREEN2,8,7,6), 
                      SSD1283A_GUI(CS_SCREEN3,8,7,6), 
                      SSD1283A_GUI(CS_SCREEN4,8,7,6) };

//SSD1283A_GUI mylcd1(/*CS=10*/ CS_SCREEN1, /*DC=*/ 8, /*RST=*/ 7, /*LED=*/ 6); //hardware spi,cs,cd,reset,led
//SSD1283A_GUI mylcd2(/*CS=10*/ CS_SCREEN2, /*DC=*/ 8, /*RST=*/ 7, /*LED=*/ 6); //hardware spi,cs,cd,reset,led
//SSD1283A_GUI mylcd3(/*CS=10*/ CS_SCREEN3, /*DC=*/ 8, /*RST=*/ 7, /*LED=*/ 6); //hardware spi,cs,cd,reset,led
//SSD1283A_GUI mylcd4(/*CS=10*/ CS_SCREEN4, /*DC=*/ 8, /*RST=*/ 7, /*LED=*/ 6); //hardware spi,cs,cd,reset,led
#endif


#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// Pick one or more screens to write to
void sel_screen(int n) {
  if(n&1) digitalWrite(CS_SCREEN1,0); else digitalWrite(CS_SCREEN1,1);
  if(n&2) digitalWrite(CS_SCREEN2,0); else digitalWrite(CS_SCREEN2,1);
  if(n&4) digitalWrite(CS_SCREEN3,0); else digitalWrite(CS_SCREEN3,1);
  if(n&8) digitalWrite(CS_SCREEN4,0); else digitalWrite(CS_SCREEN4,1);
}

void setup() 
{
  SerialIDshow(115200); // starts Serial.

  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  //if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  // Since we do not set NORMAL mode, we are in loopback mode by default.
  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT);                           // Configuring pin for /INT input

  
  pinMode(CS_SCREEN1,OUTPUT);
  pinMode(CS_SCREEN2,OUTPUT);
  pinMode(CS_SCREEN3,OUTPUT);
  pinMode(CS_SCREEN4,OUTPUT);
  sel_screen(0); // de-select all
  
  // MUST Init all at the same time
  sel_screen(1+2+4+8); // select all
  scrn[0].init();
  scrn[0].Fill_Screen(BLACK);
  sel_screen(0);
}

uint8_t last_err=0;
uint8_t last_dve=0;

void pnd(uint8_t s,uint8_t err,uint8_t dve) {
  pnd2(s,last_err,last_dve,0); // un-draw old
  pnd2(s,err,dve,1); // draw new
}

void pnd2(uint8_t s,uint8_t err,uint8_t dve, uint8_t draw) {
  uint8_t c,i; int x,y;
  char pnd[]={'P','N','D'};
  
  // Draw P, N, or D to show the transmission state:-
  for(c=0,x=0,y=9;c<3;c++) {
    scrn[s].Set_Text_Back_colour(BLACK);
    if(c==dve) {scrn[s].Set_Text_Size(4); y=y-8;}
    else scrn[s].Set_Text_Size(3);
    
    if(!draw) scrn[s].Set_Text_colour(BLACK);
    else if(c==dve)  scrn[s].Set_Text_colour(0,192,192);
    else      scrn[s].Set_Text_colour(WHITE);

    scrn[s].Set_Text_Cousur(x,y);
    scrn[s].writec(pnd[c]); // Print_String("", 45, 49);
    
    if(c==dve) {y=y+8; x+=4*6;}
    else x+=3*6; 
  }
  
    
    
    // Check Engine
    if(err) {
      uint8_t sz=2;
      if(draw) scrn[s].Set_Text_colour(RED);
      else     scrn[s].Set_Text_colour(BLACK);
      scrn[s].Set_Text_Back_colour(BLACK);
      scrn[s].Set_Text_Size(sz);
      for(c=128,x=45,y=79;c<132;c++,x+=5*sz) {
        scrn[s].Set_Text_Cousur(x,y);
        scrn[s].writec(c); // Print_String("", 45, 49);
        //scrn[s].Print_String("", 45, 49+8);
      }
      for(c=132,x=45,y=79+8*sz;c<136;c++,x+=5*sz) {
        scrn[s].Set_Text_Cousur(x,y);
        scrn[s].writec(c); // Print_String("", 45, 49);
        //scrn[s].Print_String("", 45, 49+8);
      }
    }
}


int r=0;
unsigned char bright=0;
void loop() 
{
  for(int s=0; s<4; s++) {
    int sbit=1<<s;
    sel_screen(sbit);
    scrn[s].setRotation(r);
  
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

    pnd(s,  1,1); // 0, (no error) 1, (error)  ,0 (P) ,1 (N), ,2 (D)

  } // s
  r++;

  analogWrite(6,bright); // This is how you change the LCD brightness from no backlight (0) to full on (255)
  bright+=32;
  if(bright==128)bright--; // so we hit 255 later
  if(bright<32)bright=0;   // so we hit 0 as well
  // $c=0;while(1){$c=0 if($c<32); print "$c "; $c+=32; $c-- if($c==128); $c-=256 if($c>255);}

  for(int ii=0;ii<3;ii++) {

  if(!digitalRead(CAN0_INT))                          // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);              // Read data: len = data length, buf = data byte(s)
    
    if((rxId & 0x80000000) == 0x80000000)             // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  
    Serial.print(msgString);
  
    if((rxId & 0x40000000) == 0x40000000){            // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }
        
    Serial.println();
  }else {
    //Serial.print("pin "); Serial.print(CAN0_INT); Serial.println(" is high: no data to read");
  }
  
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




  }// for //delay(3000);
}
