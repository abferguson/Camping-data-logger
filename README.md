# Camping-data-logger
A battery operated device that stores and allows realtime interaction temperature, humidity, barometric pressure, GPS coordinate and weather prediction data. The Zambretti algorithm for short term weather forecasting is used.  

This repository contains information necessary to build an interactive data logger: CAD files, circuitry and software. Both an OLED and epaper display version are presented here. Any files representing v10 is in reference to the OLED version.

I've made many camping trips into wilderness areas. While an inexpensive plastic thermometer is handy to know the current temperature, more data can be quite helpful. It can be used for selection of clothing as well as post-trip curiosity. This project is my first attempt at creating a data logger for camping.

This device uses the ESP32 mainly for its built-in Bluetooth capability. This feature allows you to use a cell phone to query the data logger (Bluetooth serial) for saved data. The sensors used are the BME280 and BN-880 GPS modules. While powered with a lithium polymer batter, inclusion of the TP4056 provides solar recharging. An onboard OLED or epaper display provides current conditions.

Firmware development: Arduino IDE

Hardware:  
1 x ESP32 MCU + female headers  
1 x BME280 module (I2C) * female header  
1 x BN-880 module (Amazon https://smile.amazon.com/gp/product/B07TY7PL54)    
1 x 132x64 0.96 inch OLED module  (version 10)
1 x 2.9inch e-Paper Display Module, 296x128 (version 11)
1 x Micro SD card module  
1 x LiPo battery (1000 mAh,Amazon https://smile.amazon.com/gp/product/B07TWHH6VF)   
1 x TP4056 battery charge module  
2 x 2N2222 transistor  
2 x MCP1700-3302E linear power regulator  
2 x SPDT slide switch (https://smile.amazon.com/gp/product/B01M1CU2B0)  
1 x power plug adapter (Amazon https://smile.amazon.com/gp/product/B089M7B481) 
various resistors and capacitors  
1 x Circuit Setup protoboard (Amazon https://smile.amazon.com/gp/product/B07HNKJNK3) 
2 x Solar cells (Amazon https://smile.amazon.com/gp/product/B0736W4HK1)  
Several JST connectors  

Circuit schematics: KiCAD format  
One of the linear power regulators provides 3.3v power to the GPS unit. The other powers everything else.  
The transistors are used to switch on power to the SD and GPS units when required. Otherwise, they are off.  

Features:  
- Power switch (inside of main body connects battery to circuitry)  
- Slide switch inside slide cover turns on OLED and places unit in real time display mode  
- Power port for solar cell supply  

Operation:  
-  Once the main power switch is on, the unit begins recording environmental and GPS data every ten minutes.
-  When the interactive slide switch, located behind the sliding cover on the side, is on, two modes are activated:
    1) The unit retrieves the lates environmental data, along with Zambretti forecast and battery voltage level, and displays on the OLED display.
    2) Bluetooth is turned on for interactive queries.  
-  Turn interactive mode off to conserve battery power. The data recording function is disabled when in interactive mode.  
-  The solar cells can be connected at any time.  
  
Bluetooth:  
-  Use a serial bluetooth app to communicate with the data logger.  One app I use is the Serial Bluetooth Terminal by Kai Morich ( Android v1.33 at time of writing)  

-  Once Bluetooth connectivity is established, sending a keyword from a mobile device to the logger will result in the logger returning a data set. This could be current conditions, stored data values, help instructions or miscellaneous data.  
   
Query   --->    Send keyword  
-Environmental --->  env  
-GPS coord --->      gps  
-Current Env --->    now  
-Help --->           help  
-Misc data --->      misc  
-Erase stored data ---> erase  

Notes:  
4 - 3mm x 6mm screws needed to attach side base to case bottom  
2 - 2mm x 5mm screws attach solar cell case bezels to solar cell case frame  
3 - 3mm x 6mm screws to attach battery cage to inside frame  
2 - 2mm x 5mm screws attache SD module to inside frame  
LiPo battery cage is designed for 504040 size  

Assembly:  
- LiPo battery is captured by battery cage, screw fastened  
- TP4056 snops onto inside frame  
- OLED or epaper displays and GPS modules snap onto case top  
- SD module is fastened to inside frame with screws  
- BME280 module soldered to female header (2.54 mm) and plugs into a pin header (male)  

Wiring: 
Using JST 2.0 connectors, I tried to bring I2C and power to each switch, power plug or sensor so that the unit could be more easily assembled/disassembled. 
The battery has its own connector allowing to completly disconnect the battery when not in use. Otherwise, the TP4056 module is still drawing power from the battery even when the system power switch is off.  
The main power switch is on the circuit board along with the discreet resistors, capacitors, transistors and linear power regulators. 
The GPS unit came with its own wiring harness and connector. The wires from the harness are soldered directly to the circuit board.  
The BME280 is soldered to a female pin header. This header fits inside a rectangulare opening allowing a male pin header to connect to it inside the main box.  




