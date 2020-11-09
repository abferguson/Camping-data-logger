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

Circuite file: KiCAD format
