# Camping-data-logger
A battery operated device that stores and allows realtime interaction temperature, humidity, barometric pressure, GPS coordinate and weather prediction data.

This repository contains information necessary to build an interactive data logger: CAD files, circuitry and software. This is the first version of this device. If I refine the design I will post on Github.

I've made many camping trips into wilderness areas. While an inexpensive plastic thermometer is handy to know the current temperature, more data can be quite helpful. It can be used for selection of clothing as well as post-trip curiosity. This project is my first attempt at creating a data logger for camping.

This device uses the ESP32 mainly for its built-in Bluetooth capability. This feature allows you to use a cell phone to query the data logger (Bluetooth serial) for saved data. The sensors used are the BME280 and BN-880 GPS modules. While powered with a lithium polymer batter, inclusion of the TP4056 provides solar recharging. An onboard OLED display provides current conditions.

Firmware development: Arduino IDE

Hardware:
1 x ESP32 MCU
1 x BME280 module
1 x BN-880 module
1 x 132x64 0.96 inch OLED module
1 x Micro SD card module
1 x LiPo battery
1 x TP4056 battery charge module
2 x 2N2222 transistor
2 x MCP1700-3302E linear power regulator
2 x micro slide switch
1 x power plug adapter
various resistors and capacitors
1 x Circuit Setup protoboard (Amazon)
Several JST connectors

Circuit schematic: KiCAD format

Features:
Power switch (inside to connect battery to circuitry)
Slide switch to turn on OLED and place unit in real time sense and display mode
Power port for solar cell supply

Notes:

4 - 3mm x 6mm screws needed to attach side base to case bottom
2 - 2mm x 5mm screws attach solar cell case bezels to solar cell case frame
3 - 3mm x 6mm screws to attach battery cage to inside frame
2 - 2mm x 5mm screws attache SD module to inside frame
LiPo battery cage is designed for 504040 size

Assembly:
LiPo battery is captured by battery cage, screw fastened
TP4056, OLED display and GPS modules snap onto either inside frame or case top
SD module is fastened to inside frame with screws
BME280 module soldered to female header (2.54 mm) and plugs into a pin header (male)

