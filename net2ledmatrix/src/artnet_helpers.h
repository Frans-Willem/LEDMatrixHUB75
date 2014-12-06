#ifndef ARTNET_HELPERS_H
#define ARTNET_HELPERS_H
#include <stdio.h>

#define ARTNET_OpOutput	0x5000

extern const char artnet_header[8];

unsigned short decode_artnet_uint16_hilo(unsigned char *data);
unsigned short decode_artnet_uint16_lohi(unsigned char *data);
unsigned int decode_artnet_uint32_hilo(unsigned char *data);
unsigned int decode_artnet_uint8(unsigned char *data);
void encode_artnet_uint16_hilo(unsigned short n, unsigned char *data);
void encode_artnet_uint16_lohi(unsigned short n, unsigned char *data);
void encode_artnet_uint32_hilo(unsigned short n, unsigned char *data);
void encode_artnet_uint32_lohi(unsigned short n, unsigned char *data);
#endif//ARTNET_HELPERS_H
