#include "artnet_helpers.h"

const char artnet_header[8]={'A','r','t','-','N','e','t',0};

unsigned short decode_artnet_uint16_hilo(unsigned char *data) {
	unsigned short ret;
	ret = data[0];
	ret <<= 8;
	ret |= data[1];
	return ret;
}

unsigned short decode_artnet_uint16_lohi(unsigned char *data) {
	return (data[1] << 8) | data[0];
}

unsigned int decode_artnet_uint32_hilo(unsigned char *data) {
	return (data[0] << 24)|(data[1] << 16)|(data[2] << 8)|data[3];
}

unsigned int decode_artnet_uint8(unsigned char *data) {
	return *(unsigned char *)data;
}

void encode_artnet_uint16_hilo(unsigned short n, unsigned char *data) {
	data[0] = (n >> 8)&0xFF;
	data[1] = n & 0xFF;
}

void encode_artnet_uint16_lohi(unsigned short n, unsigned char *data) {
	data[0] = n & 0xFF;
	data[1] = (n >> 8)&0xFF;
}

void encode_artnet_uint32_hilo(unsigned short n, unsigned char *data) {
	data[0] = (n >> 24)&0xFF;
	data[1] = (n >> 16)&0xFF;
	data[2] = (n >> 8)&0xFF;
	data[3] = (n >> 0)&0xFF;
}

void encode_artnet_uint32_lohi(unsigned short n, unsigned char *data) {
	data[0] = (n >> 0)&0xFF;
	data[1] = (n >> 8)&0xFF;
	data[2] = (n >> 16)&0xFF;
	data[3] = (n >> 24)&0xFF;
}