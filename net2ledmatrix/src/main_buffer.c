#include <stdio.h>
#include <sys/types.h>
#include <libusb.h>

#include <sys/socket.h>
#include <netdb.h>
#include <string.h>

#include "artnet_helpers.h"

#define MATRIX_WIDTH	96
#define MATRIX_HEIGHT	48

#define PIXELS_PER_UNIVERSE	170

#define PIXELS_PER_TRANSFER	20

#define MIN(a,b) ((a)<(b)?(a):(b))

libusb_device_handle *device_handle;
int socket_fd;

int matrix_init() {
	int r;
	r = libusb_init(NULL);
	if (r < 0) {
		printf("libusb_init failed\r\n");
		return r;
	}
	
	device_handle = libusb_open_device_with_vid_pid(NULL, 5824, 1500);
	if (device_handle == NULL) {
		printf("Unable to open USB device\r\n");
		libusb_exit(NULL);
		return -1;
	}
	
	if (libusb_claim_interface(device_handle, 0) != 0) {
		printf("Unable to claim USB device\r\n");
		libusb_close(device_handle);
		libusb_exit(NULL);
		return -1;
	}
}

void matrix_deinit() {
	libusb_release_interface(device_handle, 0);
	libusb_close(device_handle);
	libusb_exit(NULL);
}

int socket_init() {
	struct addrinfo hints;
	memset(&hints,0,sizeof(hints));
	hints.ai_family=AF_UNSPEC;
	hints.ai_socktype=SOCK_DGRAM;
	hints.ai_protocol=0;
	hints.ai_flags = AI_PASSIVE|AI_ADDRCONFIG|AI_NUMERICSERV;
	struct addrinfo *res;
	if (getaddrinfo(0,"6454",&hints,&res)!=0) {
		printf("Unable to look up address info\r\n");
		return -1;
	}
	printf("Open socket\r\n");
	socket_fd = socket(res->ai_family,res->ai_socktype,res->ai_protocol);
	if (socket_fd == -1) {
		printf("Unable to open socket\r\n");
		freeaddrinfo(res);
		return -1;
	}
	if (bind(socket_fd,res->ai_addr,res->ai_addrlen)==-1) {
		printf("Unable to bind socket\r\n");
		close(socket_fd);
		freeaddrinfo(res);
		return -1;
	}
	freeaddrinfo(res);
	return 0;
}

void socket_deinit() {
	close(socket_fd);
}

unsigned char output_buffer[MATRIX_WIDTH*MATRIX_HEIGHT*3];

void do_output() {
	unsigned int offset;
	unsigned char block[2 + (PIXELS_PER_TRANSFER * 3)];
	unsigned int numpixels = MATRIX_WIDTH * MATRIX_HEIGHT;
	int blocklen;
	for (offset = 0; offset<numpixels; offset+=PIXELS_PER_TRANSFER) {
		//block=&data[(blockoffset * 3)-2]; //Yes, we'll be writing out of bounds of data, at -2, but that should be safe given that it's an artnet message.
		blocklen = (MIN(numpixels - offset, PIXELS_PER_TRANSFER) * 3);
		memcpy(&block[2], &output_buffer[offset*3], blocklen);
		blocklen+=2;
		block[0] = offset >> 8;
		block[1] = offset & 0xFF;
		libusb_bulk_transfer(device_handle, 1, block, blocklen, &blocklen, 1);
	}

	libusb_control_transfer(device_handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE, 0, 0, 0, NULL, 0, 1);
}

int handle_opoutput(unsigned char Sequence, unsigned char Physical, unsigned short Universe, unsigned short Length, unsigned char *data) {
	unsigned int offset = Universe * PIXELS_PER_UNIVERSE;
	unsigned int numpixels = Length / 3;
	if (offset + numpixels > MATRIX_WIDTH * MATRIX_HEIGHT) {
		if (offset > MATRIX_WIDTH * MATRIX_HEIGHT)
			numpixels=0;
		else
			numpixels = MATRIX_WIDTH * MATRIX_HEIGHT - offset;
	}
	memcpy(&output_buffer[offset*3],data,numpixels*3);
	
	if (offset + numpixels == MATRIX_WIDTH * MATRIX_HEIGHT) {
		printf("Do output!\r\n");
		do_output();
	}
	return 0;
}

int handle_packet(struct sockaddr *addr, socklen_t addr_len, unsigned char *buffer, unsigned int bufferlen) {
	if (bufferlen < sizeof(artnet_header) || memcmp(buffer,artnet_header,sizeof(artnet_header)) != 0) {
		printf("Not an Art-Net message\r\n");
		return 0;
	}
	if (bufferlen < sizeof(artnet_header) + 4) {
		printf("Not enough arguments to Art-Net message\r\n");
		return 0;
	}
	unsigned short OpCode = decode_artnet_uint16_lohi(&buffer[sizeof(artnet_header)]);
	unsigned short ProtVer = decode_artnet_uint16_hilo(&buffer[sizeof(artnet_header) + 2]);
	if (ProtVer != 14) {
		printf("Unknown Art-Net version: %d\r\n", ProtVer);
		return 0;
	}
	if (OpCode == ARTNET_OpOutput) {
		if (bufferlen < sizeof(artnet_header) + 10) {
			printf("Not enough arguments to OpOutput\r\n");
			return 0;
		}
		unsigned char Sequence = decode_artnet_uint8(&buffer[sizeof(artnet_header) + 4]);
		unsigned char Physical = decode_artnet_uint8(&buffer[sizeof(artnet_header) + 5]);
		unsigned short Universe = decode_artnet_uint8(&buffer[sizeof(artnet_header) + 6]);
		unsigned short Length = decode_artnet_uint16_hilo(&buffer[sizeof(artnet_header) + 8]);
		if (bufferlen < sizeof(artnet_header) + 10 + Length) {
			printf("Not enough data to OpOutput %d %d %d\r\n",Length,bufferlen,bufferlen-(sizeof(artnet_header)+10));
			return 0;
		}
		return handle_opoutput(Sequence, Physical, Universe, Length, &buffer[sizeof(artnet_header)+10]);
	}
	return 0;
}

int socket_handle() {
	char buffer[1024];
	struct sockaddr_storage src_addr;
	socklen_t src_addr_len;
	src_addr_len = sizeof(src_addr);
	ssize_t count=recvfrom(socket_fd,buffer,sizeof(buffer),0,(struct sockaddr*)&src_addr,&src_addr_len);
	if (count == -1) {
		printf("Uh-oh!\r\n");
		return -1;
	}
	int retval=0;
	retval = handle_packet((struct sockaddr *)&src_addr,src_addr_len,(unsigned char *)buffer,count);
	return retval;
}

int main(void)
{
	if (matrix_init() < 0) {
		printf("Unable to open Matrix\r\n");
		return -1;
	} else {
		if (socket_init() < 0) {
			printf("Unable to open socket\r\n");
			return -1;
		} else {
			while (socket_handle() >= 0);
			socket_deinit();
		}
		matrix_deinit();
	}
	return 0;
}