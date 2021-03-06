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
L Connector_Generic:Conn_02x08_Top_Bottom J1
U 1 1 5FCE93CF
P 10050 1600
F 0 "J1" H 10100 2117 50  0000 C CNN
F 1 "Touchpad" H 10100 2026 50  0000 C CNN
F 2 "trackpad:trackpad_rect" H 10050 1600 50  0001 C CNN
F 3 "~" H 10050 1600 50  0001 C CNN
	1    10050 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10350 1300 10400 1300
Text Label 10400 1300 0    50   ~ 0
TX0
Wire Wire Line
	10350 1400 10400 1400
Text Label 10400 1400 0    50   ~ 0
TX1
Wire Wire Line
	10350 1500 10400 1500
Text Label 10400 1500 0    50   ~ 0
TX2
Wire Wire Line
	10350 1600 10400 1600
Text Label 10400 1600 0    50   ~ 0
TX3
Wire Wire Line
	10350 1700 10400 1700
Text Label 10400 1700 0    50   ~ 0
TX4
Wire Wire Line
	10350 1800 10400 1800
Text Label 10400 1800 0    50   ~ 0
TX5
Wire Wire Line
	10350 1900 10400 1900
Text Label 10400 1900 0    50   ~ 0
TX6
Wire Wire Line
	10350 2000 10400 2000
Text Label 10400 2000 0    50   ~ 0
TX7
Wire Wire Line
	9850 2000 9800 2000
Wire Wire Line
	9850 1900 9800 1900
Wire Wire Line
	9850 1800 9800 1800
Wire Wire Line
	9850 1700 9800 1700
Wire Wire Line
	9850 1600 9800 1600
Wire Wire Line
	9850 1500 9800 1500
Wire Wire Line
	9850 1400 9800 1400
Wire Wire Line
	9850 1300 9800 1300
$Comp
L Touchpad:iqs572 U1
U 1 1 5FCFDCDF
P 8750 2600
F 0 "U1" H 8725 4475 50  0000 C CNN
F 1 "iqs572" H 8725 4384 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-28_4x4mm_P0.5mm" H 8750 2450 50  0001 C CNN
F 3 "" H 8750 2450 50  0001 C CNN
	1    8750 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5FD001D3
P 7500 1700
F 0 "R2" V 7600 1650 50  0000 C CNN
F 1 "4.7k" V 7600 1850 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 7500 1700 50  0001 C CNN
F 3 "~" H 7500 1700 50  0001 C CNN
	1    7500 1700
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5FD0100C
P 7500 1600
F 0 "R1" V 7400 1550 50  0000 C CNN
F 1 "4.7k" V 7400 1750 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 7500 1600 50  0001 C CNN
F 3 "~" H 7500 1600 50  0001 C CNN
	1    7500 1600
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5FD011B7
P 6750 1150
F 0 "C2" H 6842 1196 50  0000 L CNN
F 1 "1uF" H 6842 1105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6750 1150 50  0001 C CNN
F 3 "~" H 6750 1150 50  0001 C CNN
	1    6750 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5FD0185A
P 6350 1150
F 0 "C1" H 6442 1196 50  0000 L CNN
F 1 "100pF" H 6442 1105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6350 1150 50  0001 C CNN
F 3 "~" H 6350 1150 50  0001 C CNN
	1    6350 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5FD0314B
P 7600 1150
F 0 "C4" H 7692 1196 50  0000 L CNN
F 1 "1uF" H 7692 1105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7600 1150 50  0001 C CNN
F 3 "~" H 7600 1150 50  0001 C CNN
	1    7600 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5FD03151
P 7200 1150
F 0 "C3" H 7292 1196 50  0000 L CNN
F 1 "100pF" H 7292 1105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7200 1150 50  0001 C CNN
F 3 "~" H 7200 1150 50  0001 C CNN
	1    7200 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 1600 8250 1600
Wire Wire Line
	7600 1700 8250 1700
Wire Wire Line
	7400 1600 7200 1600
Wire Wire Line
	7200 1600 7200 1550
Wire Wire Line
	7400 1700 7200 1700
Wire Wire Line
	7200 1700 7200 1600
Connection ~ 7200 1600
Wire Wire Line
	8250 1250 7850 1250
Wire Wire Line
	7600 1250 7200 1250
Connection ~ 7600 1250
Wire Wire Line
	7600 1050 8000 1050
Wire Wire Line
	8000 1050 8000 1150
Wire Wire Line
	8000 1150 8250 1150
Wire Wire Line
	7600 1050 7200 1050
Connection ~ 7600 1050
Wire Wire Line
	8250 1050 8150 1050
Wire Wire Line
	8150 1050 8150 1000
Wire Wire Line
	8150 1000 7050 1000
Wire Wire Line
	6750 1000 6750 1050
Wire Wire Line
	6350 1000 6350 1050
Wire Wire Line
	6350 1000 6750 1000
Connection ~ 6750 1000
Wire Wire Line
	6350 1250 6550 1250
Wire Wire Line
	6550 1250 6550 1300
Connection ~ 6550 1250
Wire Wire Line
	6550 1250 6750 1250
Wire Wire Line
	7850 1250 7850 1300
Connection ~ 7850 1250
Wire Wire Line
	7850 1250 7600 1250
$Comp
L power:GND #PWR0101
U 1 1 5FD0C980
P 7850 1300
F 0 "#PWR0101" H 7850 1050 50  0001 C CNN
F 1 "GND" H 7855 1127 50  0000 C CNN
F 2 "" H 7850 1300 50  0001 C CNN
F 3 "" H 7850 1300 50  0001 C CNN
	1    7850 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5FD0CD56
P 6550 1300
F 0 "#PWR0102" H 6550 1050 50  0001 C CNN
F 1 "GND" H 6555 1127 50  0000 C CNN
F 2 "" H 6550 1300 50  0001 C CNN
F 3 "" H 6550 1300 50  0001 C CNN
	1    6550 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0103
U 1 1 5FD0D347
P 7050 850
F 0 "#PWR0103" H 7050 700 50  0001 C CNN
F 1 "+3V3" H 7065 1023 50  0000 C CNN
F 2 "" H 7050 850 50  0001 C CNN
F 3 "" H 7050 850 50  0001 C CNN
	1    7050 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 1000 7050 850 
Connection ~ 7050 1000
Wire Wire Line
	7050 1000 6750 1000
Wire Wire Line
	9200 950  9250 950 
Text Label 9250 950  0    50   ~ 0
RX0
Wire Wire Line
	9200 1050 9250 1050
Text Label 9250 1050 0    50   ~ 0
RX1
Wire Wire Line
	9200 1150 9250 1150
Text Label 9250 1150 0    50   ~ 0
RX2
Wire Wire Line
	9200 1250 9250 1250
Text Label 9250 1250 0    50   ~ 0
RX3
Wire Wire Line
	9200 1350 9250 1350
Text Label 9250 1350 0    50   ~ 0
RX4
Wire Wire Line
	9200 1450 9250 1450
Wire Wire Line
	9200 1550 9250 1550
Text Label 9250 1550 0    50   ~ 0
RX6
Wire Wire Line
	9200 1650 9250 1650
Text Label 9250 1650 0    50   ~ 0
RX7
Text Label 9250 1450 0    50   ~ 0
RX5
Wire Wire Line
	9200 1850 9250 1850
Text Label 9250 1850 0    50   ~ 0
TX0
Wire Wire Line
	9200 1950 9250 1950
Text Label 9250 1950 0    50   ~ 0
TX1
Wire Wire Line
	9200 2050 9250 2050
Text Label 9250 2050 0    50   ~ 0
TX2
Wire Wire Line
	9200 2150 9250 2150
Text Label 9250 2150 0    50   ~ 0
TX3
Wire Wire Line
	9200 2250 9250 2250
Text Label 9250 2250 0    50   ~ 0
TX4
Wire Wire Line
	9200 2350 9250 2350
Text Label 9250 2350 0    50   ~ 0
TX5
Wire Wire Line
	9200 2450 9250 2450
Text Label 9250 2450 0    50   ~ 0
TX6
Wire Wire Line
	9200 2550 9250 2550
Text Label 9250 2550 0    50   ~ 0
TX7
Wire Wire Line
	9200 2650 9250 2650
$Comp
L power:+3V3 #PWR0104
U 1 1 5FD27E6B
P 7200 1550
F 0 "#PWR0104" H 7200 1400 50  0001 C CNN
F 1 "+3V3" H 7215 1723 50  0000 C CNN
F 2 "" H 7200 1550 50  0001 C CNN
F 3 "" H 7200 1550 50  0001 C CNN
	1    7200 1550
	1    0    0    -1  
$EndComp
Text Label 7900 1600 0    50   ~ 0
SDA
Text Label 7900 1700 0    50   ~ 0
SCL
Text Label 8050 1800 2    50   ~ 0
RDY
Wire Wire Line
	8050 1800 8250 1800
Wire Wire Line
	8150 2400 8250 2400
Wire Wire Line
	8150 2300 8250 2300
Wire Wire Line
	8150 2200 8250 2200
Wire Notes Line
	6150 600  10700 600 
Wire Notes Line
	10700 600  10700 2950
Wire Notes Line
	10700 2950 6150 2950
Wire Notes Line
	6150 600  6150 2950
$Comp
L Device:LED D1
U 1 1 5FD70D9B
P 7200 4500
F 0 "D1" V 7250 4350 50  0000 C CNN
F 1 "LED" V 7150 4350 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 7200 4500 50  0001 C CNN
F 3 "~" H 7200 4500 50  0001 C CNN
	1    7200 4500
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5FD71D4A
P 7550 4500
F 0 "D2" V 7600 4350 50  0000 C CNN
F 1 "LED" V 7500 4350 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 7550 4500 50  0001 C CNN
F 3 "~" H 7550 4500 50  0001 C CNN
	1    7550 4500
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D3
U 1 1 5FD73C46
P 7950 4500
F 0 "D3" V 8000 4350 50  0000 C CNN
F 1 "LED" V 7900 4350 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 7950 4500 50  0001 C CNN
F 3 "~" H 7950 4500 50  0001 C CNN
	1    7950 4500
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D4
U 1 1 5FD757FF
P 8350 4500
F 0 "D4" V 8400 4350 50  0000 C CNN
F 1 "LED" V 8300 4350 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 8350 4500 50  0001 C CNN
F 3 "~" H 8350 4500 50  0001 C CNN
	1    8350 4500
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D5
U 1 1 5FD774F2
P 8750 4500
F 0 "D5" V 8800 4350 50  0000 C CNN
F 1 "LED" V 8700 4350 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 8750 4500 50  0001 C CNN
F 3 "~" H 8750 4500 50  0001 C CNN
	1    8750 4500
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D6
U 1 1 5FD7A274
P 7200 5650
F 0 "D6" V 7250 5500 50  0000 C CNN
F 1 "LED" V 7150 5500 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 7200 5650 50  0001 C CNN
F 3 "~" H 7200 5650 50  0001 C CNN
	1    7200 5650
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D7
U 1 1 5FD7A27A
P 7600 5650
F 0 "D7" V 7650 5500 50  0000 C CNN
F 1 "LED" V 7550 5500 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 7600 5650 50  0001 C CNN
F 3 "~" H 7600 5650 50  0001 C CNN
	1    7600 5650
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D8
U 1 1 5FD7A280
P 8000 5650
F 0 "D8" V 8050 5500 50  0000 C CNN
F 1 "LED" V 7950 5500 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 8000 5650 50  0001 C CNN
F 3 "~" H 8000 5650 50  0001 C CNN
	1    8000 5650
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D9
U 1 1 5FD7A286
P 8400 5650
F 0 "D9" V 8450 5500 50  0000 C CNN
F 1 "LED" V 8350 5500 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 8400 5650 50  0001 C CNN
F 3 "~" H 8400 5650 50  0001 C CNN
	1    8400 5650
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D10
U 1 1 5FD7A28C
P 8800 5650
F 0 "D10" V 8850 5500 50  0000 C CNN
F 1 "LED" V 8750 5500 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 8800 5650 50  0001 C CNN
F 3 "~" H 8800 5650 50  0001 C CNN
	1    8800 5650
	0    1    -1   0   
$EndComp
$Comp
L Device:C_Small C5
U 1 1 5FD99B8B
P 1350 2400
F 0 "C5" H 1442 2446 50  0000 L CNN
F 1 "0.1uF" H 1442 2355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1350 2400 50  0001 C CNN
F 3 "~" H 1350 2400 50  0001 C CNN
	1    1350 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5FD9B9B1
P 650 2400
F 0 "C6" H 742 2446 50  0000 L CNN
F 1 "10uF" H 742 2355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 650 2400 50  0001 C CNN
F 3 "~" H 650 2400 50  0001 C CNN
	1    650  2400
	1    0    0    -1  
$EndComp
$Comp
L Device:Battery_Cell BT1
U 1 1 5FD9E5C0
P 3550 7300
F 0 "BT1" H 3668 7396 50  0000 L CNN
F 1 "Battery_Cell" H 3668 7305 50  0000 L CNN
F 2 "Battery:BatteryHolder_Keystone_3034_1x20mm" V 3550 7360 50  0001 C CNN
F 3 "~" V 3550 7360 50  0001 C CNN
	1    3550 7300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5FDA9EA2
P 1650 1650
F 0 "#PWR0105" H 1650 1400 50  0001 C CNN
F 1 "GND" V 1655 1522 50  0000 R CNN
F 2 "" H 1650 1650 50  0001 C CNN
F 3 "" H 1650 1650 50  0001 C CNN
	1    1650 1650
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 1700 2500 1400
Wire Wire Line
	1800 3000 1500 3000
Wire Wire Line
	1300 3000 1200 3000
Wire Wire Line
	1200 3000 1200 3100
Wire Wire Line
	1200 3100 1800 3100
$Comp
L Device:C_Small C8
U 1 1 5FDBD5ED
P 1400 3000
F 0 "C8" V 1171 3000 50  0000 C CNN
F 1 "270pF" V 1262 3000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1400 3000 50  0001 C CNN
F 3 "~" H 1400 3000 50  0001 C CNN
	1    1400 3000
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5FDC6FC9
P 1400 3200
F 0 "C9" V 1350 2900 50  0000 C CNN
F 1 "270pF" V 1450 2850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1400 3200 50  0001 C CNN
F 3 "~" H 1400 3200 50  0001 C CNN
	1    1400 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 3200 1500 3200
Wire Wire Line
	1300 3200 1200 3200
Wire Wire Line
	1200 3200 1200 3300
Wire Wire Line
	1200 3300 1800 3300
$Comp
L Device:C_Small C10
U 1 1 5FDCDE3A
P 1800 1400
F 0 "C10" V 1571 1400 50  0000 C CNN
F 1 "10nF" V 1662 1400 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1800 1400 50  0001 C CNN
F 3 "~" H 1800 1400 50  0001 C CNN
	1    1800 1400
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C11
U 1 1 5FDCF9FD
P 2150 1300
F 0 "C11" V 1950 1300 50  0000 C CNN
F 1 "3nF" V 2050 1300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2150 1300 50  0001 C CNN
F 3 "~" H 2150 1300 50  0001 C CNN
	1    2150 1300
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5FDD0800
P 2150 1550
F 0 "R4" V 2050 1500 50  0000 C CNN
F 1 "20k" V 2050 1650 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 2150 1550 50  0001 C CNN
F 3 "~" H 2150 1550 50  0001 C CNN
	1    2150 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	1650 1700 1650 1650
Wire Wire Line
	2300 1700 2300 1550
Wire Wire Line
	2300 1550 2250 1550
Wire Wire Line
	2300 1550 2300 1300
Wire Wire Line
	2300 1300 2250 1300
Connection ~ 2300 1550
Wire Wire Line
	2050 1550 2000 1550
Wire Wire Line
	2000 1550 2000 1400
Wire Wire Line
	2000 1300 2050 1300
Wire Wire Line
	1900 1400 2000 1400
Connection ~ 2000 1400
Wire Wire Line
	2000 1400 2000 1300
Wire Wire Line
	2200 1700 2200 1650
Wire Wire Line
	2200 1650 2000 1650
Wire Wire Line
	2000 1650 2000 1550
Connection ~ 2000 1550
Wire Wire Line
	1700 1400 1650 1400
Wire Wire Line
	1650 1400 1650 1650
Connection ~ 1650 1650
$Comp
L power:+3V3 #PWR0106
U 1 1 5FDF4EB0
P 1650 2250
F 0 "#PWR0106" H 1650 2100 50  0001 C CNN
F 1 "+3V3" H 1665 2423 50  0000 C CNN
F 2 "" H 1650 2250 50  0001 C CNN
F 3 "" H 1650 2250 50  0001 C CNN
	1    1650 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 2250 1650 2600
Wire Wire Line
	1650 2600 1800 2600
Wire Wire Line
	1650 2600 1650 2800
Wire Wire Line
	1650 2800 1800 2800
Connection ~ 1650 2600
Wire Wire Line
	1650 2800 1650 2900
Wire Wire Line
	1650 2900 1800 2900
Connection ~ 1650 2800
$Comp
L power:+3V3 #PWR0107
U 1 1 5FE07C4C
P 750 3350
F 0 "#PWR0107" H 750 3200 50  0001 C CNN
F 1 "+3V3" H 765 3523 50  0000 C CNN
F 2 "" H 750 3350 50  0001 C CNN
F 3 "" H 750 3350 50  0001 C CNN
	1    750  3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R3
U 1 1 5FE08223
P 1250 3400
F 0 "R3" V 1150 3350 50  0000 C CNN
F 1 "10k" V 1150 3500 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 1250 3400 50  0001 C CNN
F 3 "~" H 1250 3400 50  0001 C CNN
	1    1250 3400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	750  3350 750  3400
Wire Wire Line
	750  3400 1150 3400
Wire Wire Line
	1350 3400 1800 3400
$Comp
L power:+3V3 #PWR0108
U 1 1 5FE114F9
P 2700 4800
F 0 "#PWR0108" H 2700 4650 50  0001 C CNN
F 1 "+3V3" H 2715 4973 50  0000 C CNN
F 2 "" H 2700 4800 50  0001 C CNN
F 3 "" H 2700 4800 50  0001 C CNN
	1    2700 4800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2700 4800 2700 4700
$Comp
L power:+3V3 #PWR0109
U 1 1 5FE163A4
P 4000 3600
F 0 "#PWR0109" H 4000 3450 50  0001 C CNN
F 1 "+3V3" H 4015 3773 50  0000 C CNN
F 2 "" H 4000 3600 50  0001 C CNN
F 3 "" H 4000 3600 50  0001 C CNN
	1    4000 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 3600 3850 3600
$Comp
L power:+3V3 #PWR0110
U 1 1 5FE22F39
P 3300 1500
F 0 "#PWR0110" H 3300 1350 50  0001 C CNN
F 1 "+3V3" H 3315 1673 50  0000 C CNN
F 2 "" H 3300 1500 50  0001 C CNN
F 3 "" H 3300 1500 50  0001 C CNN
	1    3300 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 1700 3300 1550
$Comp
L power:+3V3 #PWR0111
U 1 1 5FE30689
P 2400 1700
F 0 "#PWR0111" H 2400 1550 50  0001 C CNN
F 1 "+3V3" H 2415 1873 50  0000 C CNN
F 2 "" H 2400 1700 50  0001 C CNN
F 3 "" H 2400 1700 50  0001 C CNN
	1    2400 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 1300 2400 1150
$Comp
L Device:Crystal_GND24_Small Y1
U 1 1 5FE41D21
P 2500 1300
F 0 "Y1" V 2454 1444 50  0000 L CNN
F 1 "40MHz" V 2545 1444 50  0000 L CNN
F 2 "trackpad:Oscillator_SMD_suntsu_ASCO-4Pin_1.6x1.2mm" H 2500 1300 50  0001 C CNN
F 3 "~" H 2500 1300 50  0001 C CNN
	1    2500 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 1200 2500 1100
Wire Wire Line
	2500 1100 2650 1100
Wire Wire Line
	2650 1100 2650 1450
Wire Wire Line
	2650 1450 2600 1450
Wire Wire Line
	2600 1450 2600 1700
Wire Wire Line
	2600 1300 2600 1150
Wire Wire Line
	2600 1150 2400 1150
$Comp
L power:GND #PWR0112
U 1 1 5FE4EF27
P 2400 1150
F 0 "#PWR0112" H 2400 900 50  0001 C CNN
F 1 "GND" H 2405 977 50  0000 C CNN
F 2 "" H 2400 1150 50  0001 C CNN
F 3 "" H 2400 1150 50  0001 C CNN
	1    2400 1150
	-1   0    0    1   
$EndComp
Connection ~ 2400 1150
$Comp
L Device:C_Small C7
U 1 1 5FE561FB
P 1000 2400
F 0 "C7" H 1092 2446 50  0000 L CNN
F 1 "1uF" H 1092 2355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1000 2400 50  0001 C CNN
F 3 "~" H 1000 2400 50  0001 C CNN
	1    1000 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 2300 1350 2250
Wire Wire Line
	1350 2250 1650 2250
Connection ~ 1650 2250
Wire Wire Line
	1350 2250 1000 2250
Wire Wire Line
	1000 2250 1000 2300
Connection ~ 1350 2250
Wire Wire Line
	650  2300 650  2250
Wire Wire Line
	650  2250 1000 2250
Connection ~ 1000 2250
Wire Wire Line
	650  2500 650  2550
Wire Wire Line
	650  2550 1000 2550
Wire Wire Line
	1000 2550 1000 2500
Wire Wire Line
	1000 2550 1350 2550
Wire Wire Line
	1350 2550 1350 2500
Connection ~ 1000 2550
Wire Wire Line
	1000 2550 1000 2600
$Comp
L power:GND #PWR0113
U 1 1 5FE7BE79
P 1000 2600
F 0 "#PWR0113" H 1000 2350 50  0001 C CNN
F 1 "GND" H 1005 2427 50  0000 C CNN
F 2 "" H 1000 2600 50  0001 C CNN
F 3 "" H 1000 2600 50  0001 C CNN
	1    1000 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C14
U 1 1 5FE7C8B3
P 3850 3700
F 0 "C14" H 3942 3746 50  0000 L CNN
F 1 "1uF" H 3942 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3850 3700 50  0001 C CNN
F 3 "~" H 3850 3700 50  0001 C CNN
	1    3850 3700
	1    0    0    -1  
$EndComp
Connection ~ 3850 3600
Wire Wire Line
	3850 3600 3600 3600
$Comp
L power:GND #PWR0114
U 1 1 5FE7CFCC
P 3850 3800
F 0 "#PWR0114" H 3850 3550 50  0001 C CNN
F 1 "GND" H 3855 3627 50  0000 C CNN
F 2 "" H 3850 3800 50  0001 C CNN
F 3 "" H 3850 3800 50  0001 C CNN
	1    3850 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C12
U 1 1 5FE7D3BD
P 2750 5000
F 0 "C12" H 2842 5046 50  0000 L CNN
F 1 "0.1uF" H 2842 4955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2750 5000 50  0001 C CNN
F 3 "~" H 2750 5000 50  0001 C CNN
	1    2750 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4700 2750 4700
Wire Wire Line
	2750 4700 2750 4900
Connection ~ 2700 4700
Wire Wire Line
	2700 4700 2700 4600
$Comp
L power:GND #PWR0115
U 1 1 5FE88CFA
P 2750 5100
F 0 "#PWR0115" H 2750 4850 50  0001 C CNN
F 1 "GND" H 2755 4927 50  0000 C CNN
F 2 "" H 2750 5100 50  0001 C CNN
F 3 "" H 2750 5100 50  0001 C CNN
	1    2750 5100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C13
U 1 1 5FE89351
P 3550 1650
F 0 "C13" H 3642 1696 50  0000 L CNN
F 1 "0.1uF" H 3642 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3550 1650 50  0001 C CNN
F 3 "~" H 3550 1650 50  0001 C CNN
	1    3550 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 1550 3550 1550
Connection ~ 3300 1550
Wire Wire Line
	3300 1550 3300 1500
$Comp
L power:GND #PWR0116
U 1 1 5FE8FA1E
P 3550 1750
F 0 "#PWR0116" H 3550 1500 50  0001 C CNN
F 1 "GND" H 3555 1577 50  0000 C CNN
F 2 "" H 3550 1750 50  0001 C CNN
F 3 "" H 3550 1750 50  0001 C CNN
	1    3550 1750
	1    0    0    -1  
$EndComp
Text Label 3000 1700 1    50   ~ 0
U0RXD
Text Label 2900 1700 1    50   ~ 0
U0TXD
Text Label 10100 5700 2    50   ~ 0
U0TXD
Text Label 10100 5600 2    50   ~ 0
U0RXD
Text Notes 6200 700  0    50   ~ 0
Touchpad
Text Notes 6250 4050 0    50   ~ 0
LED Driver
Text Notes 3000 7000 0    50   ~ 0
Battery
$Comp
L power:GND #PWR0117
U 1 1 5FEBABE7
P 3550 7400
F 0 "#PWR0117" H 3550 7150 50  0001 C CNN
F 1 "GND" H 3555 7227 50  0000 C CNN
F 2 "" H 3550 7400 50  0001 C CNN
F 3 "" H 3550 7400 50  0001 C CNN
	1    3550 7400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0118
U 1 1 5FEBB094
P 3550 7100
F 0 "#PWR0118" H 3550 6950 50  0001 C CNN
F 1 "+3V3" H 3565 7273 50  0000 C CNN
F 2 "" H 3550 7100 50  0001 C CNN
F 3 "" H 3550 7100 50  0001 C CNN
	1    3550 7100
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0119
U 1 1 5FEBBE51
P 9800 5500
F 0 "#PWR0119" H 9800 5350 50  0001 C CNN
F 1 "+3V3" H 9815 5673 50  0000 C CNN
F 2 "" H 9800 5500 50  0001 C CNN
F 3 "" H 9800 5500 50  0001 C CNN
	1    9800 5500
	1    0    0    -1  
$EndComp
Text Notes 10000 3350 0    50   ~ 0
Programming Header
Wire Wire Line
	9800 5500 10100 5500
Text Label 1500 3400 0    50   ~ 0
CHIP_PU
Text Label 3800 4850 0    50   ~ 0
GPI0_BOOT
Wire Wire Line
	1650 1700 2100 1700
$Comp
L SamacSys_Parts:ESP32-D0WDQ6-V3 IC1
U 1 1 5FD63BBB
P 1800 2600
F 0 "IC1" H 1700 3350 50  0000 L CNN
F 1 "ESP32-D0WDQ6-V3" H 1200 3250 50  0000 L CNN
F 2 "SamacSys_Parts:QFN40P600X600X90-49N-D_vias" H 3450 3300 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/ESP32-D0WDQ6-V3.pdf" H 3450 3200 50  0001 L CNN
F 4 "IC RF TXRX+MCU BLUETOOTH WIFI" H 3450 3100 50  0001 L CNN "Description"
F 5 "0.9" H 3450 3000 50  0001 L CNN "Height"
F 6 "356-ESP32-D0WDQ6-V3" H 3450 2900 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Espressif-Systems/ESP32-D0WDQ6-V3?qs=GBLSl2Akirt8fvUVSeeceA%3D%3D" H 3450 2800 50  0001 L CNN "Mouser Price/Stock"
F 8 "Espressif Systems" H 3450 2700 50  0001 L CNN "Manufacturer_Name"
F 9 "ESP32-D0WDQ6-V3" H 3450 2600 50  0001 L CNN "Manufacturer_Part_Number"
	1    1800 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 4600 3100 4850
Wire Wire Line
	3100 4850 3650 4850
Text Label 2300 4600 3    50   ~ 0
SDA
Text Label 2400 4600 3    50   ~ 0
SCL
Text Label 2200 4600 3    50   ~ 0
RDY
$Comp
L Switch:SW_Push SW1
U 1 1 5FFB22BC
P 4600 2500
F 0 "SW1" V 4650 2700 50  0000 C CNN
F 1 "SW_Push" V 4500 2750 50  0000 C CNN
F 2 "trackpad:SW_pts526_1.5mm" H 4600 2693 50  0001 C CNN
F 3 "~" H 4600 2700 50  0001 C CNN
	1    4600 2500
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C16
U 1 1 6005CC3D
P 6600 1950
F 0 "C16" H 6692 1996 50  0000 L CNN
F 1 "22uF" H 6692 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6600 1950 50  0001 C CNN
F 3 "~" H 6600 1950 50  0001 C CNN
	1    6600 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C17
U 1 1 6005D6C9
P 7000 1950
F 0 "C17" H 7092 1996 50  0000 L CNN
F 1 "22uF" H 7092 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7000 1950 50  0001 C CNN
F 3 "~" H 7000 1950 50  0001 C CNN
	1    7000 1950
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0125
U 1 1 6005DBBF
P 6800 1750
F 0 "#PWR0125" H 6800 1600 50  0001 C CNN
F 1 "+3V3" H 6815 1923 50  0000 C CNN
F 2 "" H 6800 1750 50  0001 C CNN
F 3 "" H 6800 1750 50  0001 C CNN
	1    6800 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 1750 6800 1850
Wire Wire Line
	6800 1850 6600 1850
Wire Wire Line
	6800 1850 7000 1850
Connection ~ 6800 1850
Wire Wire Line
	6600 2050 6600 2100
Wire Wire Line
	6600 2100 6800 2100
Wire Wire Line
	7000 2100 7000 2050
Wire Wire Line
	6800 2100 6800 2150
Connection ~ 6800 2100
Wire Wire Line
	6800 2100 7000 2100
$Comp
L power:GND #PWR0126
U 1 1 600798D3
P 6800 2150
F 0 "#PWR0126" H 6800 1900 50  0001 C CNN
F 1 "GND" H 6805 1977 50  0000 C CNN
F 2 "" H 6800 2150 50  0001 C CNN
F 3 "" H 6800 2150 50  0001 C CNN
	1    6800 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 6009A6AE
P 10850 5500
F 0 "#PWR0128" H 10850 5250 50  0001 C CNN
F 1 "GND" H 10855 5327 50  0000 C CNN
F 2 "" H 10850 5500 50  0001 C CNN
F 3 "" H 10850 5500 50  0001 C CNN
	1    10850 5500
	0    -1   -1   0   
$EndComp
Text Label 10100 5800 2    50   ~ 0
SDA
Text Label 10100 5900 2    50   ~ 0
SCL
Text Label 10600 5800 0    50   ~ 0
RDY
$Comp
L Device:R_Small R6
U 1 1 5FDF061D
P 1850 5950
F 0 "R6" V 1750 5900 50  0000 C CNN
F 1 "10k" V 1750 6050 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 1850 5950 50  0001 C CNN
F 3 "~" H 1850 5950 50  0001 C CNN
	1    1850 5950
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R7
U 1 1 5FDF1540
P 3400 5950
F 0 "R7" V 3300 5900 50  0000 C CNN
F 1 "10k" V 3300 6050 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 3400 5950 50  0001 C CNN
F 3 "~" H 3400 5950 50  0001 C CNN
	1    3400 5950
	0    1    1    0   
$EndComp
Text Label 10600 5600 0    50   ~ 0
DTR
Text Label 10600 5700 0    50   ~ 0
RTS
Text Label 3750 5950 0    50   ~ 0
DTR
Text Label 1600 5950 2    50   ~ 0
RTS
Wire Wire Line
	1950 5950 2250 5950
Wire Wire Line
	1600 5950 1700 5950
Wire Wire Line
	3150 5950 3300 5950
Wire Wire Line
	3500 5950 3650 5950
$Comp
L SamacSys_Parts:MBT3946DW1T1G Q1
U 1 1 5FE22FF0
P 2250 5850
F 0 "Q1" H 2700 6115 50  0000 C CNN
F 1 "MBT6429DW1T1G" H 2700 6024 50  0000 C CNN
F 2 "SOT65P210X110-6N" H 3000 5950 50  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/MBT6429DW1T1-D.PDF" H 3000 5850 50  0001 L CNN
F 4 "" H 3000 5750 50  0001 L CNN "Description"
F 5 "1.1" H 3000 5650 50  0001 L CNN "Height"
F 6 "863-MBT3946DW1T1G" H 3000 5550 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/ON-Semiconductor/MBT3946DW1T1G?qs=3JMERSakebo8moNowe%2F%2FpA%3D%3D" H 3000 5450 50  0001 L CNN "Mouser Price/Stock"
F 8 "ON Semiconductor" H 3000 5350 50  0001 L CNN "Manufacturer_Name"
F 9 "MBT3946DW1T1G" H 3000 5250 50  0001 L CNN "Manufacturer_Part_Number"
	1    2250 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 6050 3150 6250
Wire Wire Line
	3150 6250 1700 6250
Wire Wire Line
	1700 6250 1700 5950
Connection ~ 1700 5950
Wire Wire Line
	1700 5950 1750 5950
Wire Wire Line
	2250 5850 2100 5850
Wire Wire Line
	2100 5850 2100 5400
Wire Wire Line
	2100 5400 3650 5400
Wire Wire Line
	3650 5400 3650 5950
Connection ~ 3650 5950
Wire Wire Line
	3650 5950 3750 5950
Text Label 2250 6050 2    50   ~ 0
CHIP_PU
Wire Wire Line
	3150 5750 3150 5850
Text Label 3150 5750 0    50   ~ 0
GPI0_BOOT
Wire Wire Line
	10850 5500 10600 5500
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J2
U 1 1 5FEA1B85
P 10300 5700
F 0 "J2" H 10350 6117 50  0000 C CNN
F 1 "Programming Header" H 10350 6026 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical_SMD" H 10300 5700 50  0001 C CNN
F 3 "~" H 10300 5700 50  0001 C CNN
	1    10300 5700
	1    0    0    -1  
$EndComp
Text Label 9800 1800 2    50   ~ 0
RX5
Text Label 9800 2000 2    50   ~ 0
RX7
Text Label 9800 1900 2    50   ~ 0
RX6
Text Label 9800 1700 2    50   ~ 0
RX4
Text Label 9800 1600 2    50   ~ 0
RX3
Text Label 9800 1500 2    50   ~ 0
RX2
Text Label 9800 1400 2    50   ~ 0
RX1
Text Label 9800 1300 2    50   ~ 0
RX0
$Comp
L Device:R_Small R8
U 1 1 5FE3BA28
P 4600 2000
F 0 "R8" V 4500 1950 50  0000 C CNN
F 1 "10k" V 4500 2100 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 4600 2000 50  0001 C CNN
F 3 "~" H 4600 2000 50  0001 C CNN
	1    4600 2000
	-1   0    0    1   
$EndComp
Text Label 5350 2250 2    50   ~ 0
SW_CALC
Text Label 4600 2250 2    50   ~ 0
SW_RST
Wire Wire Line
	4600 2100 4600 2300
$Comp
L power:+3V3 #PWR0120
U 1 1 5FE4C9BE
P 4600 1900
F 0 "#PWR0120" H 4600 1750 50  0001 C CNN
F 1 "+3V3" H 4615 2073 50  0000 C CNN
F 2 "" H 4600 1900 50  0001 C CNN
F 3 "" H 4600 1900 50  0001 C CNN
	1    4600 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 5FE4CF43
P 4600 2700
F 0 "#PWR0127" H 4600 2450 50  0001 C CNN
F 1 "GND" H 4605 2527 50  0000 C CNN
F 2 "" H 4600 2700 50  0001 C CNN
F 3 "" H 4600 2700 50  0001 C CNN
	1    4600 2700
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 5FE56D9E
P 5350 2500
F 0 "SW2" V 5400 2700 50  0000 C CNN
F 1 "SW_Push" V 5250 2750 50  0000 C CNN
F 2 "trackpad:SW_pts526_1.5mm" H 5350 2693 50  0001 C CNN
F 3 "~" H 5350 2700 50  0001 C CNN
	1    5350 2500
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R9
U 1 1 5FE56DA4
P 5350 2000
F 0 "R9" V 5250 1950 50  0000 C CNN
F 1 "10k" V 5250 2100 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 5350 2000 50  0001 C CNN
F 3 "~" H 5350 2000 50  0001 C CNN
	1    5350 2000
	-1   0    0    1   
$EndComp
Wire Wire Line
	5350 2100 5350 2300
$Comp
L power:+3V3 #PWR0129
U 1 1 5FE56DAC
P 5350 1900
F 0 "#PWR0129" H 5350 1750 50  0001 C CNN
F 1 "+3V3" H 5365 2073 50  0000 C CNN
F 2 "" H 5350 1900 50  0001 C CNN
F 3 "" H 5350 1900 50  0001 C CNN
	1    5350 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0130
U 1 1 5FE56DB2
P 5350 2700
F 0 "#PWR0130" H 5350 2450 50  0001 C CNN
F 1 "GND" H 5355 2527 50  0000 C CNN
F 2 "" H 5350 2700 50  0001 C CNN
F 3 "" H 5350 2700 50  0001 C CNN
	1    5350 2700
	1    0    0    -1  
$EndComp
Text Label 3200 1700 1    50   ~ 0
SW_RST
Text Label 3100 1700 1    50   ~ 0
SW_CALC
Text Label 3600 3000 0    50   ~ 0
D1
Text Label 3600 3100 0    50   ~ 0
D2
Text Label 3600 3300 0    50   ~ 0
D4
Text Label 3600 3200 0    50   ~ 0
D3
Text Label 3600 3400 0    50   ~ 0
D5
Text Label 3600 3500 0    50   ~ 0
D6
Text Label 3600 3700 0    50   ~ 0
D7
Text Label 3200 4600 3    50   ~ 0
D8
Text Label 3000 4600 3    50   ~ 0
D9
Text Label 3600 2900 0    50   ~ 0
D0
$Comp
L power:GND #PWR0131
U 1 1 5FF12883
P 7200 4650
F 0 "#PWR0131" H 7200 4400 50  0001 C CNN
F 1 "GND" H 7205 4477 50  0000 C CNN
F 2 "" H 7200 4650 50  0001 C CNN
F 3 "" H 7200 4650 50  0001 C CNN
	1    7200 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0132
U 1 1 5FF12DE8
P 7550 4650
F 0 "#PWR0132" H 7550 4400 50  0001 C CNN
F 1 "GND" H 7555 4477 50  0000 C CNN
F 2 "" H 7550 4650 50  0001 C CNN
F 3 "" H 7550 4650 50  0001 C CNN
	1    7550 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0133
U 1 1 5FF1301A
P 7950 4650
F 0 "#PWR0133" H 7950 4400 50  0001 C CNN
F 1 "GND" H 7955 4477 50  0000 C CNN
F 2 "" H 7950 4650 50  0001 C CNN
F 3 "" H 7950 4650 50  0001 C CNN
	1    7950 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0134
U 1 1 5FF132E9
P 8350 4650
F 0 "#PWR0134" H 8350 4400 50  0001 C CNN
F 1 "GND" H 8355 4477 50  0000 C CNN
F 2 "" H 8350 4650 50  0001 C CNN
F 3 "" H 8350 4650 50  0001 C CNN
	1    8350 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0135
U 1 1 5FF13641
P 8750 4650
F 0 "#PWR0135" H 8750 4400 50  0001 C CNN
F 1 "GND" H 8755 4477 50  0000 C CNN
F 2 "" H 8750 4650 50  0001 C CNN
F 3 "" H 8750 4650 50  0001 C CNN
	1    8750 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0136
U 1 1 5FF1388E
P 7200 5800
F 0 "#PWR0136" H 7200 5550 50  0001 C CNN
F 1 "GND" H 7205 5627 50  0000 C CNN
F 2 "" H 7200 5800 50  0001 C CNN
F 3 "" H 7200 5800 50  0001 C CNN
	1    7200 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0137
U 1 1 5FF13CEC
P 7600 5800
F 0 "#PWR0137" H 7600 5550 50  0001 C CNN
F 1 "GND" H 7605 5627 50  0000 C CNN
F 2 "" H 7600 5800 50  0001 C CNN
F 3 "" H 7600 5800 50  0001 C CNN
	1    7600 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0138
U 1 1 5FF13FAF
P 8000 5800
F 0 "#PWR0138" H 8000 5550 50  0001 C CNN
F 1 "GND" H 8005 5627 50  0000 C CNN
F 2 "" H 8000 5800 50  0001 C CNN
F 3 "" H 8000 5800 50  0001 C CNN
	1    8000 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0139
U 1 1 5FF142AF
P 8400 5800
F 0 "#PWR0139" H 8400 5550 50  0001 C CNN
F 1 "GND" H 8405 5627 50  0000 C CNN
F 2 "" H 8400 5800 50  0001 C CNN
F 3 "" H 8400 5800 50  0001 C CNN
	1    8400 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0140
U 1 1 5FF146CE
P 8800 5800
F 0 "#PWR0140" H 8800 5550 50  0001 C CNN
F 1 "GND" H 8805 5627 50  0000 C CNN
F 2 "" H 8800 5800 50  0001 C CNN
F 3 "" H 8800 5800 50  0001 C CNN
	1    8800 5800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R10
U 1 1 5FF1FCCB
P 7200 4200
F 0 "R10" H 7050 4250 50  0000 C CNN
F 1 "330" H 7050 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 7200 4200 50  0001 C CNN
F 3 "~" H 7200 4200 50  0001 C CNN
	1    7200 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 4300 7200 4350
Wire Wire Line
	7200 4100 7200 4050
$Comp
L Device:R_Small R12
U 1 1 5FF33884
P 7550 4200
F 0 "R12" H 7400 4250 50  0000 C CNN
F 1 "330" H 7400 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 7550 4200 50  0001 C CNN
F 3 "~" H 7550 4200 50  0001 C CNN
	1    7550 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 4300 7550 4350
Wire Wire Line
	7550 4100 7550 4050
$Comp
L Device:R_Small R14
U 1 1 5FF3C213
P 7950 4200
F 0 "R14" H 7800 4250 50  0000 C CNN
F 1 "330" H 7800 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 7950 4200 50  0001 C CNN
F 3 "~" H 7950 4200 50  0001 C CNN
	1    7950 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 4300 7950 4350
Wire Wire Line
	7950 4100 7950 4050
$Comp
L Device:R_Small R16
U 1 1 5FF450AE
P 8350 4200
F 0 "R16" H 8200 4250 50  0000 C CNN
F 1 "330" H 8200 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 8350 4200 50  0001 C CNN
F 3 "~" H 8350 4200 50  0001 C CNN
	1    8350 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 4300 8350 4350
Wire Wire Line
	8350 4100 8350 4050
$Comp
L Device:R_Small R18
U 1 1 5FF4DBF5
P 8750 4200
F 0 "R18" H 8600 4250 50  0000 C CNN
F 1 "330" H 8600 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 8750 4200 50  0001 C CNN
F 3 "~" H 8750 4200 50  0001 C CNN
	1    8750 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 4300 8750 4350
Wire Wire Line
	8750 4100 8750 4050
$Comp
L Device:R_Small R11
U 1 1 5FF56920
P 7200 5350
F 0 "R11" H 7050 5400 50  0000 C CNN
F 1 "330" H 7050 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 7200 5350 50  0001 C CNN
F 3 "~" H 7200 5350 50  0001 C CNN
	1    7200 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 5450 7200 5500
Wire Wire Line
	7200 5250 7200 5200
$Comp
L Device:R_Small R13
U 1 1 5FF5F9D9
P 7600 5350
F 0 "R13" H 7450 5400 50  0000 C CNN
F 1 "330" H 7450 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 7600 5350 50  0001 C CNN
F 3 "~" H 7600 5350 50  0001 C CNN
	1    7600 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 5450 7600 5500
Wire Wire Line
	7600 5250 7600 5200
$Comp
L Device:R_Small R15
U 1 1 5FF72807
P 8000 5350
F 0 "R15" H 7850 5400 50  0000 C CNN
F 1 "330" H 7850 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 8000 5350 50  0001 C CNN
F 3 "~" H 8000 5350 50  0001 C CNN
	1    8000 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 5450 8000 5500
Wire Wire Line
	8000 5250 8000 5200
$Comp
L Device:R_Small R17
U 1 1 5FF7BC50
P 8400 5350
F 0 "R17" H 8250 5400 50  0000 C CNN
F 1 "330" H 8250 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 8400 5350 50  0001 C CNN
F 3 "~" H 8400 5350 50  0001 C CNN
	1    8400 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 5450 8400 5500
Wire Wire Line
	8400 5250 8400 5200
$Comp
L Device:R_Small R19
U 1 1 5FF851F1
P 8800 5350
F 0 "R19" H 8650 5400 50  0000 C CNN
F 1 "330" H 8650 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 8800 5350 50  0001 C CNN
F 3 "~" H 8800 5350 50  0001 C CNN
	1    8800 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 5450 8800 5500
Wire Wire Line
	8800 5250 8800 5200
Text Label 7200 4050 2    50   ~ 0
D1
Text Label 7550 4050 2    50   ~ 0
D2
Text Label 7950 4050 2    50   ~ 0
D3
Text Label 8350 4050 2    50   ~ 0
D4
Text Label 8750 4050 2    50   ~ 0
D5
Text Label 7200 5200 2    50   ~ 0
D6
Text Label 7600 5200 2    50   ~ 0
D7
Text Label 8000 5200 2    50   ~ 0
D8
Text Label 8400 5200 2    50   ~ 0
D9
Text Label 8800 5200 2    50   ~ 0
D0
NoConn ~ 1800 2700
NoConn ~ 1800 3500
NoConn ~ 1800 3600
NoConn ~ 1800 3700
NoConn ~ 2100 4600
NoConn ~ 2500 4600
NoConn ~ 2600 4600
NoConn ~ 2800 4600
NoConn ~ 2900 4600
NoConn ~ 3600 2800
NoConn ~ 3600 2700
NoConn ~ 3600 2600
NoConn ~ 2800 1700
$Comp
L power:+3V3 #PWR0121
U 1 1 5FF70E28
P 2700 1700
F 0 "#PWR0121" H 2700 1550 50  0001 C CNN
F 1 "+3V3" H 2715 1873 50  0000 C CNN
F 2 "" H 2700 1700 50  0001 C CNN
F 3 "" H 2700 1700 50  0001 C CNN
	1    2700 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5FF74E6E
P 3650 4650
F 0 "R5" H 3550 4600 50  0000 C CNN
F 1 "10k" H 3550 4750 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 3650 4650 50  0001 C CNN
F 3 "~" H 3650 4650 50  0001 C CNN
	1    3650 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 4750 3650 4850
Connection ~ 3650 4850
Wire Wire Line
	3650 4850 3800 4850
Wire Wire Line
	3650 4550 3650 4450
$Comp
L power:+3V3 #PWR0122
U 1 1 5FFA46DC
P 3650 4450
F 0 "#PWR0122" H 3650 4300 50  0001 C CNN
F 1 "+3V3" H 3665 4623 50  0000 C CNN
F 2 "" H 3650 4450 50  0001 C CNN
F 3 "" H 3650 4450 50  0001 C CNN
	1    3650 4450
	1    0    0    -1  
$EndComp
NoConn ~ 10600 5900
NoConn ~ 8150 2200
NoConn ~ 8150 2300
NoConn ~ 8150 2400
NoConn ~ 9250 2650
$EndSCHEMATC
