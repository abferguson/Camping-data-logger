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
L Regulator_Linear:MCP1700-3302E_SOT23 U2
U 1 1 5F36B874
P 3350 5300
F 0 "U2" H 3250 5550 50  0000 C CNN
F 1 "MCP1700-3302E" H 3200 5450 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3350 5525 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826D.pdf" H 3350 5300 50  0001 C CNN
	1    3350 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5F37CEAD
P 2850 5950
F 0 "C1" H 2965 5996 50  0001 L CNN
F 1 "10uF" H 2736 5950 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2888 5800 50  0001 C CNN
F 3 "~" H 2850 5950 50  0001 C CNN
	1    2850 5950
	1    0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5F37E579
P 3800 5950
F 0 "C2" H 3915 5996 50  0000 L CNN
F 1 "10uF" H 3915 5905 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3838 5800 50  0001 C CNN
F 3 "~" H 3800 5950 50  0001 C CNN
	1    3800 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 5800 3800 5300
Connection ~ 3800 5300
$Comp
L power:GND #PWR0101
U 1 1 5F3865B6
P 3350 5600
F 0 "#PWR0101" H 3350 5350 50  0001 C CNN
F 1 "GND" H 3355 5427 50  0000 C CNN
F 2 "" H 3350 5600 50  0001 C CNN
F 3 "" H 3350 5600 50  0001 C CNN
	1    3350 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 5800 2850 5300
Wire Wire Line
	2850 5300 3050 5300
$Comp
L power:GND #PWR0102
U 1 1 5F389725
P 2850 6100
F 0 "#PWR0102" H 2850 5850 50  0001 C CNN
F 1 "GND" H 2855 5927 50  0000 C CNN
F 2 "" H 2850 6100 50  0001 C CNN
F 3 "" H 2850 6100 50  0001 C CNN
	1    2850 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5F389F0C
P 3800 6100
F 0 "#PWR0103" H 3800 5850 50  0001 C CNN
F 1 "GND" H 3805 5927 50  0000 C CNN
F 2 "" H 3800 6100 50  0001 C CNN
F 3 "" H 3800 6100 50  0001 C CNN
	1    3800 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 5300 2850 5300
Connection ~ 2850 5300
$Comp
L Regulator_Linear:MCP1700-3302E_SOT23 U4
U 1 1 5F3A2991
P 3150 2350
F 0 "U4" H 3150 2592 50  0000 C CNN
F 1 "MCP1700-3302E" H 3150 2501 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3150 2575 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826D.pdf" H 3150 2350 50  0001 C CNN
	1    3150 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5F3A3923
P 3150 2650
F 0 "#PWR0105" H 3150 2400 50  0001 C CNN
F 1 "GND" H 3155 2477 50  0000 C CNN
F 2 "" H 3150 2650 50  0001 C CNN
F 3 "" H 3150 2650 50  0001 C CNN
	1    3150 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5F3A401A
P 2450 2750
F 0 "C3" H 2565 2796 50  0000 L CNN
F 1 "100nF" H 2565 2705 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2488 2600 50  0001 C CNN
F 3 "~" H 2450 2750 50  0001 C CNN
	1    2450 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5F3A4FAD
P 2450 2900
F 0 "#PWR0106" H 2450 2650 50  0001 C CNN
F 1 "GND" H 2455 2727 50  0000 C CNN
F 2 "" H 2450 2900 50  0001 C CNN
F 3 "" H 2450 2900 50  0001 C CNN
	1    2450 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5F3A554C
P 3800 2900
F 0 "#PWR0107" H 3800 2650 50  0001 C CNN
F 1 "GND" H 3805 2727 50  0000 C CNN
F 2 "" H 3800 2900 50  0001 C CNN
F 3 "" H 3800 2900 50  0001 C CNN
	1    3800 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 2350 3800 2600
Wire Wire Line
	2850 2350 2450 2350
Wire Wire Line
	2100 2350 2100 3650
Wire Wire Line
	2450 2600 2450 2350
Connection ~ 2450 2350
Wire Wire Line
	2450 2350 2100 2350
$Comp
L Connector:Micro_SD_Card SD1
U 1 1 5F3A8AC8
P 7300 3850
F 0 "SD1" H 7250 4567 50  0000 C CNN
F 1 "Micro_SD_Card" H 7250 4476 50  0000 C CNN
F 2 "Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5" H 8450 4150 50  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/693072010801.pdf" H 7300 3850 50  0001 C CNN
	1    7300 3850
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 Batt1
U 1 1 5F3B7CE6
P 850 6800
F 0 "Batt1" H 768 7017 50  0000 C CNN
F 1 "Power" H 768 6926 50  0000 C CNN
F 2 "Connector_JST:JST_PH_S2B-PH-K_1x02_P2.00mm_Horizontal" H 850 6800 50  0001 C CNN
F 3 "~" H 850 6800 50  0001 C CNN
	1    850  6800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2500 950  2500 1550
Wire Wire Line
	8300 4450 8100 4450
Wire Wire Line
	4900 2750 8300 2750
Wire Wire Line
	8300 2750 8300 4450
Wire Wire Line
	5100 3950 4300 3950
Wire Wire Line
	4300 3950 4300 2950
$Comp
L power:GND #PWR0109
U 1 1 5F384E33
P 6300 4300
F 0 "#PWR0109" H 6300 4050 50  0001 C CNN
F 1 "GND" H 6305 4127 50  0000 C CNN
F 2 "" H 6300 4300 50  0001 C CNN
F 3 "" H 6300 4300 50  0001 C CNN
	1    6300 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 4150 5800 4150
Wire Wire Line
	6400 3950 5700 3950
Wire Wire Line
	6400 3650 5000 3650
Wire Wire Line
	6400 3750 6100 3750
$Comp
L Connector:Conn_01x04_Male BME280
U 1 1 5F38B76D
P 2600 1750
F 0 "BME280" V 2754 1462 50  0000 R CNN
F 1 "BME280" V 2663 1462 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 2600 1750 50  0001 C CNN
F 3 "~" H 2600 1750 50  0001 C CNN
	1    2600 1750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2600 1050 2600 1550
$Comp
L power:GND #PWR0112
U 1 1 5F38DCDF
P 2800 1050
F 0 "#PWR0112" H 2800 800 50  0001 C CNN
F 1 "GND" H 2805 877 50  0000 C CNN
F 2 "" H 2800 1050 50  0001 C CNN
F 3 "" H 2800 1050 50  0001 C CNN
	1    2800 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1550 2700 1400
Wire Wire Line
	2700 1400 3750 1400
Wire Wire Line
	5900 1450 3850 1450
Wire Wire Line
	2800 1450 2800 1550
Wire Wire Line
	2600 1050 2800 1050
$Comp
L Connector:Conn_01x04_Male OLED1
U 1 1 5F3921F0
P 3650 1750
F 0 "OLED1" V 3804 1462 50  0000 R CNN
F 1 "0.96\" OLED" V 3713 1462 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3650 1750 50  0001 C CNN
F 3 "~" H 3650 1750 50  0001 C CNN
	1    3650 1750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2500 950  3550 950 
Wire Wire Line
	3550 950  3550 1550
Connection ~ 2500 950 
Wire Wire Line
	3750 1550 3750 1400
Wire Wire Line
	3750 1400 6000 1400
Wire Wire Line
	3650 1550 3650 1050
Wire Wire Line
	3650 1050 3900 1050
$Comp
L power:GND #PWR0113
U 1 1 5F39C560
P 3900 1050
F 0 "#PWR0113" H 3900 800 50  0001 C CNN
F 1 "GND" H 3905 877 50  0000 C CNN
F 2 "" H 3900 1050 50  0001 C CNN
F 3 "" H 3900 1050 50  0001 C CNN
	1    3900 1050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H5
U 1 1 5F39F0FD
P 9150 1050
F 0 "H5" H 9250 1096 50  0000 L CNN
F 1 "MountingHole" H 9250 1005 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 9150 1050 50  0001 C CNN
F 3 "~" H 9150 1050 50  0001 C CNN
	1    9150 1050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H7
U 1 1 5F39F9EC
P 10200 1050
F 0 "H7" H 10300 1096 50  0000 L CNN
F 1 "MountingHole" H 10300 1005 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 10200 1050 50  0001 C CNN
F 3 "~" H 10200 1050 50  0001 C CNN
	1    10200 1050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H6
U 1 1 5F3A0372
P 9150 1600
F 0 "H6" H 9250 1646 50  0000 L CNN
F 1 "MountingHole" H 9250 1555 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 9150 1600 50  0001 C CNN
F 3 "~" H 9150 1600 50  0001 C CNN
	1    9150 1600
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H8
U 1 1 5F3A0995
P 10200 1600
F 0 "H8" H 10300 1646 50  0000 L CNN
F 1 "MountingHole" H 10300 1555 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 10200 1600 50  0001 C CNN
F 3 "~" H 10200 1600 50  0001 C CNN
	1    10200 1600
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 5F3A11AA
P 7900 1550
F 0 "H1" H 8000 1599 50  0000 L CNN
F 1 "GPS VDD" H 8000 1508 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D1.0mm_Drill0.5mm" H 7900 1550 50  0001 C CNN
F 3 "~" H 7900 1550 50  0001 C CNN
	1    7900 1550
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 5F3A1909
P 7900 1850
F 0 "H2" H 8000 1899 50  0000 L CNN
F 1 "GPS RX" H 8000 1808 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7900 1850 50  0001 C CNN
F 3 "~" H 7900 1850 50  0001 C CNN
	1    7900 1850
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 5F3A1C45
P 7900 2150
F 0 "H3" H 8000 2199 50  0000 L CNN
F 1 "GPS TX" H 8000 2108 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7900 2150 50  0001 C CNN
F 3 "~" H 7900 2150 50  0001 C CNN
	1    7900 2150
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 5F3A20CE
P 7900 2450
F 0 "H4" H 8000 2499 50  0000 L CNN
F 1 "GPS GND" H 8000 2408 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7900 2450 50  0001 C CNN
F 3 "~" H 7900 2450 50  0001 C CNN
	1    7900 2450
	1    0    0    -1  
$EndComp
Connection ~ 3750 1400
Wire Wire Line
	3850 1450 3850 1550
Connection ~ 3850 1450
Wire Wire Line
	3850 1450 2800 1450
$Comp
L Device:R R2
U 1 1 5F3B27EE
P 8600 3150
F 0 "R2" V 8393 3150 50  0000 C CNN
F 1 "470" V 8484 3150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 8530 3150 50  0001 C CNN
F 3 "~" H 8600 3150 50  0001 C CNN
	1    8600 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	3450 2350 3800 2350
Wire Wire Line
	7900 1650 6450 1650
Wire Wire Line
	6450 1650 6450 2350
Wire Wire Line
	6450 2350 3800 2350
Connection ~ 3800 2350
Wire Wire Line
	5600 1950 7900 1950
Wire Wire Line
	5500 2250 7900 2250
Wire Wire Line
	6400 4050 6300 4050
Wire Wire Line
	6300 4050 6300 4300
Wire Wire Line
	3800 3850 3800 4850
Wire Wire Line
	3800 3850 6400 3850
$Comp
L Transistor_BJT:2N3904 Q2
U 1 1 5F3C2D3E
P 9100 3150
F 0 "Q2" H 9290 3196 50  0000 L CNN
F 1 "2N2222" H 9290 3105 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9300 3075 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 9100 3150 50  0001 L CNN
	1    9100 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 3150 8900 3150
Wire Wire Line
	5200 3150 8450 3150
$Comp
L power:GND #PWR0110
U 1 1 5F3D6349
P 9200 3350
F 0 "#PWR0110" H 9200 3100 50  0001 C CNN
F 1 "GND" H 9205 3177 50  0000 C CNN
F 2 "" H 9200 3350 50  0001 C CNN
F 3 "" H 9200 3350 50  0001 C CNN
	1    9200 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5F3D6AE3
P 4900 3150
F 0 "#PWR0111" H 4900 2900 50  0001 C CNN
F 1 "GND" H 4905 2977 50  0000 C CNN
F 2 "" H 4900 3150 50  0001 C CNN
F 3 "" H 4900 3150 50  0001 C CNN
	1    4900 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 2550 9200 2550
Wire Wire Line
	9200 2550 9200 2950
$Comp
L Device:R R1
U 1 1 5F3D3E86
P 4450 2950
F 0 "R1" V 4243 2950 50  0000 C CNN
F 1 "4.7k" V 4334 2950 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 4380 2950 50  0001 C CNN
F 3 "~" H 4450 2950 50  0001 C CNN
	1    4450 2950
	0    1    1    0   
$EndComp
$Comp
L Transistor_BJT:2N3904 Q1
U 1 1 5F3C3FCF
P 4800 2950
F 0 "Q1" H 4990 2996 50  0000 L CNN
F 1 "2N2222" H 4990 2905 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5000 2875 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 4800 2950 50  0001 L CNN
	1    4800 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5F3ED9BB
P 2500 3650
F 0 "R3" V 2293 3650 50  0000 C CNN
F 1 "6.8k" V 2384 3650 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 2430 3650 50  0001 C CNN
F 3 "~" H 2500 3650 50  0001 C CNN
	1    2500 3650
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5F3EDDA1
P 3150 3650
F 0 "R4" V 2943 3650 50  0000 C CNN
F 1 "10k" V 3034 3650 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 3080 3650 50  0001 C CNN
F 3 "~" H 3150 3650 50  0001 C CNN
	1    3150 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	2350 3650 2100 3650
Connection ~ 2100 3650
Wire Wire Line
	2100 3650 2100 5300
Wire Wire Line
	3450 3650 3300 3650
$Comp
L power:GND #PWR0114
U 1 1 5F3F47C5
P 3450 3650
F 0 "#PWR0114" H 3450 3400 50  0001 C CNN
F 1 "GND" H 3455 3477 50  0000 C CNN
F 2 "" H 3450 3650 50  0001 C CNN
F 3 "" H 3450 3650 50  0001 C CNN
	1    3450 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 3650 2850 3650
Wire Wire Line
	2850 4000 2850 3650
Connection ~ 2850 3650
Wire Wire Line
	2850 3650 3000 3650
Wire Wire Line
	1750 950  1750 4850
Wire Wire Line
	1750 4850 3800 4850
Wire Wire Line
	1750 950  2500 950 
Connection ~ 3800 4850
Wire Wire Line
	3800 4850 3800 5300
$Comp
L Device:C C4
U 1 1 5F3A4975
P 3800 2750
F 0 "C4" H 3915 2796 50  0000 L CNN
F 1 "100nF" H 3915 2705 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3838 2600 50  0001 C CNN
F 3 "~" H 3800 2750 50  0001 C CNN
	1    3800 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5F4A1CFE
P 1250 4850
F 0 "#PWR0115" H 1250 4600 50  0001 C CNN
F 1 "GND" H 1255 4677 50  0000 C CNN
F 2 "" H 1250 4850 50  0001 C CNN
F 3 "" H 1250 4850 50  0001 C CNN
	1    1250 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 4800 1250 4800
Wire Wire Line
	1250 4800 1250 4850
Wire Wire Line
	1000 4600 4500 4600
Wire Wire Line
	4600 4500 1000 4500
Wire Wire Line
	4800 4400 1000 4400
Wire Wire Line
	3800 5300 3650 5300
Wire Wire Line
	1000 4700 3950 4700
Connection ~ 3950 5300
Wire Wire Line
	3800 5300 3950 5300
Wire Wire Line
	3950 5300 3950 4700
Wire Wire Line
	4100 4000 2850 4000
Wire Wire Line
	4700 6150 4100 6150
Wire Wire Line
	4100 6150 4100 4000
Wire Wire Line
	4700 5900 4700 6150
Wire Wire Line
	6100 3750 6100 4700
Wire Wire Line
	6000 1400 6000 4700
Wire Wire Line
	4600 4700 4600 4500
Wire Wire Line
	4800 4700 4800 4400
Wire Wire Line
	5900 1450 5900 4700
Wire Wire Line
	5800 4150 5800 4700
Wire Wire Line
	5700 3950 5700 4700
Wire Wire Line
	4500 5900 4500 6000
Wire Wire Line
	5000 3650 5000 4700
Wire Wire Line
	5600 4700 5600 1950
Wire Wire Line
	5500 2250 5500 4700
Wire Wire Line
	4500 4600 4500 4700
Wire Wire Line
	3950 5300 4300 5300
Wire Wire Line
	5200 4700 5200 3150
Wire Wire Line
	5100 4700 5100 3950
$Comp
L power:GND #PWR0108
U 1 1 5F3AB2AD
P 7100 5300
F 0 "#PWR0108" H 7100 5050 50  0001 C CNN
F 1 "GND" H 7105 5127 50  0000 C CNN
F 2 "" H 7100 5300 50  0001 C CNN
F 3 "" H 7100 5300 50  0001 C CNN
	1    7100 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 6000 4250 6000
Wire Wire Line
	4250 4300 1000 4300
$Comp
L RF_Module:ESP32-WROOM-32D ESP32
U 1 1 5F36C3C9
P 5700 5300
F 0 "ESP32" V 5750 3500 50  0000 R CNN
F 1 "ESP32-WROOM-32D" V 5650 3750 50  0000 R CNN
F 2 "RF_Module:ESP32-WROOM-32" V 5609 3856 50  0001 R CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32d_esp32-wroom-32u_datasheet_en.pdf" H 5400 5350 50  0001 C CNN
	1    5700 5300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4250 4300 4250 6000
$Comp
L Connector_Generic:Conn_01x06 Program1
U 1 1 5F4A314A
P 800 4500
F 0 "Program1" H 718 4917 50  0000 C CNN
F 1 "Program" H 718 4826 50  0000 C CNN
F 2 "Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical" H 800 4500 50  0001 C CNN
F 3 "~" H 800 4500 50  0001 C CNN
	1    800  4500
	-1   0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad B+1
U 1 1 5F52C6D5
P 1850 6700
F 0 "B+1" V 1804 6850 50  0000 L CNN
F 1 "Batt +" V 1895 6850 50  0000 L CNN
F 2 "Connector_Pin:Pin_D0.7mm_L6.5mm_W1.8mm_FlatFork" H 1850 6700 50  0001 C CNN
F 3 "~" H 1850 6700 50  0001 C CNN
	1    1850 6700
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad B-1
U 1 1 5F52CAB7
P 1850 7150
F 0 "B-1" V 1804 7300 50  0000 L CNN
F 1 "Batt -" V 1895 7300 50  0000 L CNN
F 2 "Connector_Pin:Pin_D0.7mm_L6.5mm_W1.8mm_FlatFork" H 1850 7150 50  0001 C CNN
F 3 "~" H 1850 7150 50  0001 C CNN
	1    1850 7150
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad Out+1
U 1 1 5F52D11F
P 2100 5700
F 0 "Out+1" H 2150 5600 50  0000 R CNN
F 1 "Out +" H 2250 5900 50  0000 R CNN
F 2 "Connector_Pin:Pin_D0.7mm_L6.5mm_W1.8mm_FlatFork" H 2100 5700 50  0001 C CNN
F 3 "~" H 2100 5700 50  0001 C CNN
	1    2100 5700
	1    0    0    1   
$EndComp
$Comp
L Mechanical:MountingHole_Pad Out-1
U 1 1 5F52DB5C
P 1650 5800
F 0 "Out-1" H 1550 6000 50  0000 L CNN
F 1 "Out -" H 1400 5750 50  0000 L CNN
F 2 "Connector_Pin:Pin_D0.7mm_L6.5mm_W1.8mm_FlatFork" H 1650 5800 50  0001 C CNN
F 3 "~" H 1650 5800 50  0001 C CNN
	1    1650 5800
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad In+1
U 1 1 5F52DF71
P 2650 6750
F 0 "In+1" H 2750 6799 50  0000 L CNN
F 1 "In +" H 2750 6708 50  0000 L CNN
F 2 "Connector_Pin:Pin_D0.7mm_L6.5mm_W1.8mm_FlatFork" H 2650 6750 50  0001 C CNN
F 3 "~" H 2650 6750 50  0001 C CNN
	1    2650 6750
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad In-1
U 1 1 5F52E526
P 3100 6750
F 0 "In-1" H 3200 6799 50  0000 L CNN
F 1 "In -" H 3200 6708 50  0000 L CNN
F 2 "Connector_Pin:Pin_D0.7mm_L6.5mm_W1.8mm_FlatFork" H 3100 6750 50  0001 C CNN
F 3 "~" H 3100 6750 50  0001 C CNN
	1    3100 6750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5F5384F1
P 1650 6100
F 0 "#PWR0104" H 1650 5850 50  0001 C CNN
F 1 "GND" H 1655 5927 50  0000 C CNN
F 2 "" H 1650 6100 50  0001 C CNN
F 3 "" H 1650 6100 50  0001 C CNN
	1    1650 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 5900 1650 6100
Wire Wire Line
	2100 5600 2100 5300
Connection ~ 2100 5300
$Comp
L power:GND #PWR0116
U 1 1 5F578DA6
P 3100 7000
F 0 "#PWR0116" H 3100 6750 50  0001 C CNN
F 1 "GND" H 3105 6827 50  0000 C CNN
F 2 "" H 3100 7000 50  0001 C CNN
F 3 "" H 3100 7000 50  0001 C CNN
	1    3100 7000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 5F579677
P 1900 7400
F 0 "#PWR0117" H 1900 7150 50  0001 C CNN
F 1 "GND" H 1905 7227 50  0000 C CNN
F 2 "" H 1900 7400 50  0001 C CNN
F 3 "" H 1900 7400 50  0001 C CNN
	1    1900 7400
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPDT SW1
U 1 1 5F57D53E
P 1400 6800
F 0 "SW1" H 1400 7085 50  0000 C CNN
F 1 "SW_SPDT" H 1400 6994 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 1400 6800 50  0001 C CNN
F 3 "~" H 1400 6800 50  0001 C CNN
	1    1400 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 6800 1200 6800
Wire Wire Line
	1050 6900 1150 6900
Wire Wire Line
	1150 6900 1150 7150
Wire Wire Line
	1150 7150 1650 7150
Wire Wire Line
	1600 6900 1700 6900
Wire Wire Line
	1700 6900 1700 6700
Wire Wire Line
	1700 6700 1750 6700
Wire Wire Line
	3100 6850 3100 7000
Wire Wire Line
	1900 7400 1650 7400
Wire Wire Line
	1650 7400 1650 7150
Connection ~ 1650 7150
Wire Wire Line
	1650 7150 1750 7150
$EndSCHEMATC
