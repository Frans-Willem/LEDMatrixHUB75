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
| PD0 | R1 |
| PD1 | G1 |
| PD2 | B1 |
| PD3 | R1 |
| PD4 | G1 |
| PD5 | B1 |
| PE2 | A |
| PE3 | B |
| PE4 | C |
| PE5 | D (may be left unconnected for 1/8th scan) |
| PD6 | CLK |
| PE6 | STB |
| PC6 | OE |
