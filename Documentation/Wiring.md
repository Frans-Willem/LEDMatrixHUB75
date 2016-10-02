Wiring
===
HUB75 pins
---
The HUB75 pin out, on the LED matrices.

| 1 | 2 |
| --- | --- |
| R1 |  G1 |
| B1 | GND |
| R2 | G2 |
| B2 | GND |
| A | B |
| C | D |
| CLK | STB |
| OE | GND |

The LED panel on the top-right is where the signal should be piped in from the STM32F4Discovery board, and should be piped in a snake-like fashion to the rest of the panels (left on the top row, right on the bottom row (these panels should be inversed), and left again on the bottom row).

STM32F4Discovery matrix pins
---
| STM32 | Name |
| --- | --- |
| GND | GND (Don't forget to connect this!) |
| PD0 | R1 |
| PD1 | G1 |
| PD2 | B1 |
| PD3 | R2 |
| PD4 | G2 |
| PD5 | B2 |
| PE2 | A |
| PE3 | B |
| PE4 | C |
| PE5 | D (may be left unconnected for 1/8th scan) |
| PD6 | CLK |
| PE6 | STB |
| PC6 | OE |

STM32F4Discovery control pins
---
| STM32 | Name |
| --- | --- |
| GND | GND (Don't forget to connect this) |
| PB6 | TXD (Can be left unconnected) |
| PB7 | RXD (Should be connected to router's TXD) |

Note that since the WR703N has an USB port, you can easily power the STM32 board by plugging the ST-LINK connection in there.