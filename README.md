# Arduino_Pro_Mini33_SSD1283A_CAN_MultiScreen
This Arduino (atmega328p) code drives 4 or more SSD1283A LCD screens and displays data on them from CAN_Bus, serial, analog and/or digital inputs.

# Initial Release

This first release is a working proof-of-concept that reads CAN data and displays it over serial, while displaying assorted messages on all 4 LCD screens while dimming them.

Future updates will add custom routines for digital display of car-related instruments.


# Supported hardware

1. Only these 1.6" "SSD1283" screens:  https://www.aliexpress.com/wholesale?SearchText=SSD1283
 ![SSD2183 LCD Front](https://chrisdrake.com/img/lcd1.jpg) ![SSD2183 LCD Back](https://chrisdrake.com/img/lcd2.jpg)
2. Any Arduino: https://en.wikipedia.org/wiki/Arduino_Uno  (Only tested on atmega328p chips at 3.3v and 8mhz so far - e.g. Arduino "Pro Mini".
   ![Arduino Pro Mini 3.3v](https://chrisdrake.com/img/pro_mini.png)
3. Cheap CAN_Bus sheild
   ![Chinese CAN-BUS Adapter](https://chrisdrake.com/img/chinese_CAN.jpg)

# NOTICE

We are not affiliated in any way whatsoever with any hardware suppliers.  This is no commercial relationship or benefit involved at all with this code.

