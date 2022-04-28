EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Regulator_Linear:MCP1700-3302E_SOT23 SD_BME280_ESP32_Power1
U 1 1 5F36B874
P 2380 3930
F 0 "SD_BME280_ESP32_Power1" H 2440 4200 50  0000 C CNN
F 1 "MCP1700-3302E" H 2380 4080 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 2380 4155 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826D.pdf" H 2380 3930 50  0001 C CNN
	1    2380 3930
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5F37CEAD
P 1890 4350
F 0 "C1" H 2030 4320 50  0000 L CNN
F 1 "100uF" H 2230 4400 50  0000 R CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 1928 4200 50  0001 C CNN
F 3 "~" H 1890 4350 50  0001 C CNN
	1    1890 4350
	1    0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5F37E579
P 2840 4340
F 0 "C2" H 2955 4386 50  0000 L CNN
F 1 "100uF" H 2950 4300 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 2878 4190 50  0001 C CNN
F 3 "~" H 2840 4340 50  0001 C CNN
	1    2840 4340
	1    0    0    -1  
$EndComp
Wire Wire Line
	2840 4190 2840 3930
Connection ~ 2840 3930
$Comp
L power:GND #PWR0101
U 1 1 5F3865B6
P 2380 4230
F 0 "#PWR0101" H 2380 3980 50  0001 C CNN
F 1 "GND" H 2385 4057 50  0000 C CNN
F 2 "" H 2380 4230 50  0001 C CNN
F 3 "" H 2380 4230 50  0001 C CNN
	1    2380 4230
	1    0    0    -1  
$EndComp
Wire Wire Line
	1890 4200 1890 3930
Wire Wire Line
	1890 3930 2080 3930
$Comp
L power:GND #PWR0102
U 1 1 5F389725
P 1890 4500
F 0 "#PWR0102" H 1890 4250 50  0001 C CNN
F 1 "GND" H 1895 4327 50  0000 C CNN
F 2 "" H 1890 4500 50  0001 C CNN
F 3 "" H 1890 4500 50  0001 C CNN
	1    1890 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5F389F0C
P 2840 4490
F 0 "#PWR0103" H 2840 4240 50  0001 C CNN
F 1 "GND" H 2845 4317 50  0000 C CNN
F 2 "" H 2840 4490 50  0001 C CNN
F 3 "" H 2840 4490 50  0001 C CNN
	1    2840 4490
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:MCP1700-3302E_SOT23 GPS_Power1
U 1 1 5F3A2991
P 2590 2610
F 0 "GPS_Power1" H 2590 2852 50  0000 C CNN
F 1 "MCP1700-3302E" H 2590 2761 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 2590 2835 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826D.pdf" H 2590 2610 50  0001 C CNN
	1    2590 2610
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5F3A3923
P 2590 2910
F 0 "#PWR0105" H 2590 2660 50  0001 C CNN
F 1 "GND" H 2595 2737 50  0000 C CNN
F 2 "" H 2590 2910 50  0001 C CNN
F 3 "" H 2590 2910 50  0001 C CNN
	1    2590 2910
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5F3A401A
P 1890 3010
F 0 "C3" H 2005 3056 50  0000 L CNN
F 1 "100nF" H 2005 2965 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 1928 2860 50  0001 C CNN
F 3 "~" H 1890 3010 50  0001 C CNN
	1    1890 3010
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5F3A4FAD
P 1890 3160
F 0 "#PWR0106" H 1890 2910 50  0001 C CNN
F 1 "GND" H 1895 2987 50  0000 C CNN
F 2 "" H 1890 3160 50  0001 C CNN
F 3 "" H 1890 3160 50  0001 C CNN
	1    1890 3160
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5F3A554C
P 3110 3160
F 0 "#PWR0107" H 3110 2910 50  0001 C CNN
F 1 "GND" H 3115 2987 50  0000 C CNN
F 2 "" H 3110 3160 50  0001 C CNN
F 3 "" H 3110 3160 50  0001 C CNN
	1    3110 3160
	1    0    0    -1  
$EndComp
Wire Wire Line
	3110 2610 3110 2860
Wire Wire Line
	2290 2610 1890 2610
Wire Wire Line
	1890 2860 1890 2610
$Comp
L Connector_Generic:Conn_01x02 Batt1
U 1 1 5F3B7CE6
P 1860 5880
F 0 "Batt1" V 1900 6050 50  0000 C CNN
F 1 "LiPo Battery" V 1990 6020 50  0000 C CNN
F 2 "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical" H 1860 5880 50  0001 C CNN
F 3 "~" H 1860 5880 50  0001 C CNN
	1    1860 5880
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5F38DCDF
P 4590 2170
F 0 "#PWR0112" H 4590 1920 50  0001 C CNN
F 1 "GND" H 4595 1997 50  0000 C CNN
F 2 "" H 4590 2170 50  0001 C CNN
F 3 "" H 4590 2170 50  0001 C CNN
	1    4590 2170
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5F3B27EE
P 7240 5620
F 0 "R2" V 7033 5620 50  0000 C CNN
F 1 "470" V 7124 5620 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7170 5620 50  0001 C CNN
F 3 "~" H 7240 5620 50  0001 C CNN
	1    7240 5620
	0    1    1    0   
$EndComp
Wire Wire Line
	2890 2610 3110 2610
Connection ~ 3110 2610
$Comp
L Transistor_BJT:2N3904 Q2
U 1 1 5F3C2D3E
P 7690 5620
F 0 "Q2" H 7880 5666 50  0000 L CNN
F 1 "2N2222" H 7880 5575 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 7890 5545 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 7690 5620 50  0001 L CNN
	1    7690 5620
	1    0    0    -1  
$EndComp
Wire Wire Line
	7390 5620 7490 5620
$Comp
L power:GND #PWR0110
U 1 1 5F3D6349
P 7790 5820
F 0 "#PWR0110" H 7790 5570 50  0001 C CNN
F 1 "GND" H 7795 5647 50  0000 C CNN
F 2 "" H 7790 5820 50  0001 C CNN
F 3 "" H 7790 5820 50  0001 C CNN
	1    7790 5820
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5F3D6AE3
P 8200 6290
F 0 "#PWR0111" H 8200 6040 50  0001 C CNN
F 1 "GND" H 8205 6117 50  0000 C CNN
F 2 "" H 8200 6290 50  0001 C CNN
F 3 "" H 8200 6290 50  0001 C CNN
	1    8200 6290
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5F3ED9BB
P 4400 6590
F 0 "R3" V 4193 6590 50  0000 C CNN
F 1 "6.8k" V 4284 6590 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4330 6590 50  0001 C CNN
F 3 "~" H 4400 6590 50  0001 C CNN
	1    4400 6590
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5F3EDDA1
P 5050 6590
F 0 "R4" V 4843 6590 50  0000 C CNN
F 1 "10k" V 4934 6590 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4980 6590 50  0001 C CNN
F 3 "~" H 5050 6590 50  0001 C CNN
	1    5050 6590
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 6590 5200 6590
$Comp
L power:GND #PWR0114
U 1 1 5F3F47C5
P 5350 6590
F 0 "#PWR0114" H 5350 6340 50  0001 C CNN
F 1 "GND" H 5355 6417 50  0000 C CNN
F 2 "" H 5350 6590 50  0001 C CNN
F 3 "" H 5350 6590 50  0001 C CNN
	1    5350 6590
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 6590 4760 6590
Connection ~ 4760 6590
Wire Wire Line
	4760 6590 4900 6590
$Comp
L Device:C C4
U 1 1 5F3A4975
P 3110 3010
F 0 "C4" H 3225 3056 50  0000 L CNN
F 1 "100nF" H 3225 2965 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3148 2860 50  0001 C CNN
F 3 "~" H 3110 3010 50  0001 C CNN
	1    3110 3010
	1    0    0    -1  
$EndComp
Wire Wire Line
	2840 3930 2680 3930
$Comp
L power:GND #PWR0108
U 1 1 5F3AB2AD
P 4360 4400
F 0 "#PWR0108" H 4360 4150 50  0001 C CNN
F 1 "GND" H 4365 4227 50  0000 C CNN
F 2 "" H 4360 4400 50  0001 C CNN
F 3 "" H 4360 4400 50  0001 C CNN
	1    4360 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5F5384F1
P 2290 6230
F 0 "#PWR0104" H 2290 5980 50  0001 C CNN
F 1 "GND" H 2295 6057 50  0000 C CNN
F 2 "" H 2290 6230 50  0001 C CNN
F 3 "" H 2290 6230 50  0001 C CNN
	1    2290 6230
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPDT SW1
U 1 1 5F57D53E
P 1540 5710
F 0 "SW1" V 1540 5995 50  0000 C CNN
F 1 "SW_SPDT" V 1440 5990 50  0000 C CNN
F 2 "Tonys_KiCAD_footprint_library:Tony_slide_SPDT_switch" H 1540 5710 50  0001 C CNN
F 3 "~" H 1540 5710 50  0001 C CNN
	1    1540 5710
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 60D486D7
P 4590 1540
F 0 "#PWR0109" H 4590 1290 50  0001 C CNN
F 1 "GND" H 4595 1367 50  0000 C CNN
F 2 "" H 4590 1540 50  0001 C CNN
F 3 "" H 4590 1540 50  0001 C CNN
	1    4590 1540
	1    0    0    -1  
$EndComp
Wire Wire Line
	7190 2040 6160 2040
$Comp
L Device:R R1
U 1 1 5F3D3E86
P 7550 6060
F 0 "R1" V 7343 6060 50  0000 C CNN
F 1 "4.7k" V 7434 6060 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7480 6060 50  0001 C CNN
F 3 "~" H 7550 6060 50  0001 C CNN
	1    7550 6060
	0    1    -1   0   
$EndComp
$Comp
L Transistor_BJT:2N3904 Q1
U 1 1 5F3C3FCF
P 8100 6060
F 0 "Q1" H 8290 6106 50  0000 L CNN
F 1 "2N2222" H 8290 6015 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 8300 5985 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 8100 6060 50  0001 L CNN
	1    8100 6060
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 6260 8200 6290
Wire Wire Line
	7900 6060 7700 6060
Wire Wire Line
	7190 1840 6940 1840
Wire Wire Line
	7190 1940 6830 1940
Wire Wire Line
	7190 2140 6680 2140
$Comp
L Switch:SW_SPDT SW2
U 1 1 60D7FB53
P 3780 6080
F 0 "SW2" H 3780 6365 50  0000 C CNN
F 1 "Bluetooth/RTC0 Wakeup" H 3780 6274 50  0000 C CNN
F 2 "Tonys_KiCAD_footprint_library:Tony_slide_SPDT_switch" H 3780 6080 50  0001 C CNN
F 3 "~" H 3780 6080 50  0001 C CNN
	1    3780 6080
	1    0    0    1   
$EndComp
$Comp
L Device:R R5
U 1 1 60D80D24
P 4250 5980
F 0 "R5" V 4340 5990 50  0000 C CNN
F 1 "5.1k" V 4410 5980 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4180 5980 50  0001 C CNN
F 3 "~" H 4250 5980 50  0001 C CNN
	1    4250 5980
	0    1    1    0   
$EndComp
Wire Wire Line
	4100 5980 3980 5980
$Comp
L power:GND #PWR0116
U 1 1 5F578DA6
P 2500 7560
F 0 "#PWR0116" H 2500 7310 50  0001 C CNN
F 1 "GND" H 2505 7387 50  0000 C CNN
F 2 "" H 2500 7560 50  0001 C CNN
F 3 "" H 2500 7560 50  0001 C CNN
	1    2500 7560
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 60DE93A3
P 4160 3500
F 0 "#PWR0113" H 4160 3250 50  0001 C CNN
F 1 "GND" H 4165 3327 50  0000 C CNN
F 2 "" H 4160 3500 50  0001 C CNN
F 3 "" H 4160 3500 50  0001 C CNN
	1    4160 3500
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male Serial_Pins1
U 1 1 60DE863F
P 3780 3290
F 0 "Serial_Pins1" H 3880 3110 50  0000 C CNN
F 1 "115200" H 3870 3030 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3780 3290 50  0001 C CNN
F 3 "~" H 3780 3290 50  0001 C CNN
	1    3780 3290
	1    0    0    -1  
$EndComp
Wire Wire Line
	3980 3390 4160 3390
Wire Wire Line
	4160 3390 4160 3500
Connection ~ 1890 3930
Wire Wire Line
	1540 5910 1540 6420
Wire Wire Line
	2290 6230 2290 6140
Wire Wire Line
	2290 6140 2090 6140
$Comp
L data_logger-v3-rescue:ESP-32S_NodeMCU-Tonys_KiCAD_symbol_library U3
U 1 1 60E28E85
P 4660 5350
F 0 "U3" V 5160 5880 50  0000 C CNN
F 1 "ESP-32S_NodeMCU" V 5270 5880 50  0000 C CNN
F 2 "Tonys_KiCAD_footprint_library:Tony_ESP-32S_NodeMCU" H 4710 5350 50  0001 C CNN
F 3 "" H 4710 5350 50  0001 C CNN
	1    4660 5350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5860 4380 5860 4450
Wire Wire Line
	5360 5980 4520 5980
Wire Wire Line
	3980 3190 5060 3190
Wire Wire Line
	3980 3290 4960 3290
Wire Wire Line
	5460 1740 7190 1740
Wire Wire Line
	4760 1640 7190 1640
Wire Wire Line
	6160 4450 6160 2040
Wire Wire Line
	5760 5760 6940 5760
Wire Wire Line
	6940 5760 6940 1840
Wire Wire Line
	6830 1940 6830 5960
Wire Wire Line
	6830 5960 5460 5960
Wire Wire Line
	6680 2140 6680 5860
Wire Wire Line
	6680 5860 5560 5860
Wire Wire Line
	4590 1540 7190 1540
Wire Wire Line
	4360 4400 4360 4290
Wire Wire Line
	4360 4290 4660 4290
Wire Wire Line
	4660 4290 4660 4450
Wire Wire Line
	5860 6060 7400 6060
Wire Wire Line
	6060 5620 7090 5620
Wire Wire Line
	1640 3930 1890 3930
Wire Wire Line
	1640 3930 1640 5180
Connection ~ 1640 5180
Wire Wire Line
	1640 5180 1640 5510
Wire Wire Line
	4250 6590 3180 6590
Wire Wire Line
	3180 6590 3180 5180
Wire Wire Line
	3180 5180 1640 5180
Wire Wire Line
	1990 6420 1990 6140
Wire Wire Line
	2840 3930 3410 3930
Connection ~ 3410 3930
Wire Wire Line
	4400 5980 4520 5980
Wire Wire Line
	4520 5980 4520 5970
Connection ~ 4520 5980
Wire Wire Line
	3580 6080 3410 6080
Wire Wire Line
	3110 2610 4340 2610
Wire Wire Line
	7380 4570 7790 4570
Wire Wire Line
	7790 4570 7790 5420
Wire Wire Line
	1890 2610 1180 2610
Wire Wire Line
	1180 2610 1180 5180
Wire Wire Line
	1180 5180 1640 5180
Connection ~ 1890 2610
Wire Wire Line
	2090 6140 2090 6420
Wire Wire Line
	1990 6140 2090 6140
Connection ~ 2090 6140
Wire Wire Line
	1760 6080 1760 6140
Wire Wire Line
	1760 6140 1640 6140
Wire Wire Line
	1640 6140 1640 6420
Wire Wire Line
	1860 6080 1860 6140
Wire Wire Line
	1860 6140 1990 6140
Connection ~ 1990 6140
Wire Wire Line
	1540 7320 1540 7500
Wire Wire Line
	1540 7500 1310 7500
Wire Wire Line
	2090 7600 2090 7500
Wire Wire Line
	2500 7560 2500 7500
Wire Wire Line
	2500 7500 2090 7500
Connection ~ 2090 7500
Wire Wire Line
	7560 2670 7380 2670
Wire Wire Line
	7380 2670 7380 4570
Wire Wire Line
	7560 2870 5660 2870
Wire Wire Line
	7560 2970 4340 2970
Wire Wire Line
	4340 2610 4340 2970
Wire Wire Line
	7560 2770 5760 2770
Wire Wire Line
	5860 4380 4520 4380
Wire Wire Line
	4520 4380 4520 5980
Wire Wire Line
	5360 5980 5360 5500
Wire Wire Line
	5460 5500 5460 5960
Wire Wire Line
	5560 5500 5560 5860
Wire Wire Line
	5760 5500 5760 5760
Wire Wire Line
	5860 5500 5860 6060
Wire Wire Line
	4760 5500 4760 6590
Wire Wire Line
	3410 3930 3410 5670
Wire Wire Line
	4660 5500 4660 5670
Wire Wire Line
	4660 5670 3410 5670
Connection ~ 3410 5670
Wire Wire Line
	3410 5670 3410 6080
Wire Wire Line
	4960 4450 4960 3290
Wire Wire Line
	5060 4450 5060 3190
Wire Wire Line
	5460 4450 5460 4090
Wire Wire Line
	5660 2870 5660 4450
Wire Wire Line
	5760 2770 5760 4450
Wire Wire Line
	6060 5500 6060 5620
Wire Wire Line
	1310 7600 2090 7600
$Comp
L Connector:AudioJack2 Solar1
U 1 1 61084F59
P 1110 7500
F 0 "Solar1" H 931 7483 50  0000 R CNN
F 1 "AudioJack2" H 931 7574 50  0000 R CNN
F 2 "Connector_Audio:Jack_3.5mm_QingPu_WQP-PJ398SM_Vertical_CircularHoles" H 1110 7500 50  0001 C CNN
F 3 "~" H 1110 7500 50  0001 C CNN
	1    1110 7500
	1    0    0    1   
$EndComp
Wire Wire Line
	4860 4450 4860 2070
Wire Wire Line
	5160 4450 5160 1970
Wire Wire Line
	4760 4450 4760 3990
Wire Wire Line
	3410 1440 3410 2440
Wire Wire Line
	8680 4290 8200 4290
Wire Wire Line
	8200 4290 8200 5860
Wire Wire Line
	8680 4190 5360 4190
Wire Wire Line
	8680 4090 5460 4090
Connection ~ 5460 4090
Wire Wire Line
	8680 3990 4760 3990
Connection ~ 4760 3990
Wire Wire Line
	8680 3790 3410 3790
Connection ~ 3410 3790
Wire Wire Line
	3410 3790 3410 3930
Wire Wire Line
	4760 1640 4760 3990
Wire Wire Line
	5460 1740 5460 4090
Wire Wire Line
	5360 4190 5360 4450
Wire Wire Line
	8680 3890 5560 3890
Wire Wire Line
	5560 3890 5560 4450
$Comp
L Tonys_KiCAD_symbol_library:Tony_BME280 U5
U 1 1 60E32DD9
P 4180 1920
F 0 "U5" V 4005 1728 50  0000 C CNN
F 1 "Tony_BME280" V 4096 1728 50  0000 C CNN
F 2 "Tonys_KiCAD_footprint_library:Tony_BME280" V 4097 1728 50  0001 C CNN
F 3 "" H 4180 1920 50  0001 C CNN
	1    4180 1920
	0    1    1    0   
$EndComp
Wire Wire Line
	4280 2270 4420 2270
Wire Wire Line
	4420 2270 4420 2440
Wire Wire Line
	4420 2440 3410 2440
Connection ~ 3410 2440
Wire Wire Line
	3410 2440 3410 3790
Wire Wire Line
	4280 2170 4590 2170
Wire Wire Line
	4280 1970 5160 1970
Wire Wire Line
	4280 2070 4860 2070
Wire Wire Line
	3410 1440 7190 1440
$Comp
L Tonys_KiCAD_symbol_library:Tony_ANMbest_SD_connector U4
U 1 1 60E81B90
P 8780 4390
F 0 "U4" V 9176 3562 50  0000 R CNN
F 1 "Tony_ANMbest_SD_connector" V 9085 3562 50  0000 R CNN
F 2 "Tonys_KiCAD_footprint_library:Tony_ANMbest_micro_SD" H 8780 4390 50  0001 C CNN
F 3 "" H 8780 4390 50  0001 C CNN
	1    8780 4390
	0    -1   -1   0   
$EndComp
$Comp
L Tonys_KiCAD_symbol_library:Tony_Beitian_BN-880_GPS GPS1
U 1 1 60E87A65
P 8760 3370
F 0 "GPS1" H 8732 2774 50  0000 R CNN
F 1 "Tony_Beitian_BN-880_GPS" H 8732 2865 50  0000 R CNN
F 2 "Tonys_KiCAD_footprint_library:Tony_Beitian_BN-880_GPS" H 8760 3370 50  0001 C CNN
F 3 "" H 8760 3370 50  0001 C CNN
	1    8760 3370
	-1   0    0    1   
$EndComp
$Comp
L Tonys_KiCAD_symbol_library:Tony_2.9_inch_ePaper_display U2
U 1 1 60E91E95
P 7290 2190
F 0 "U2" H 8575 2661 50  0000 L CNN
F 1 "Tony_2.9_inch_ePaper_display" H 8575 2570 50  0000 L CNN
F 2 "Tonys_KiCAD_footprint_library:Tony_Waveshare_2.9_ePaper_display" H 7290 2190 50  0001 C CNN
F 3 "" H 7290 2190 50  0001 C CNN
	1    7290 2190
	1    0    0    -1  
$EndComp
$Comp
L Tonys_KiCAD_symbol_library:Tony_TP4056_batt_solar U1
U 1 1 60E929AD
P 1540 6420
F 0 "U1" H 2168 6016 50  0000 L CNN
F 1 "Tony_TP4056_batt_solar" H 2168 5925 50  0000 L CNN
F 2 "Tonys_KiCAD_footprint_library:Tony_TP4056_batt_solar" H 1540 6420 50  0001 C CNN
F 3 "" H 1540 6420 50  0001 C CNN
	1    1540 6420
	1    0    0    -1  
$EndComp
Wire Wire Line
	2090 7320 2090 7500
$EndSCHEMATC
