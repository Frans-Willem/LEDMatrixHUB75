#include <stdio.h>
#include <sys/types.h>

#include <sys/socket.h>
#include <netdb.h>
#include <string.h>

#include "artnet_helpers.h"

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b) ((a)<(b)?(b):(a))
#endif

#include <asm/termios.h>
#include <asm/ioctls.h>
#include <fcntl.h>

#define BYTES_PER_UNIVERSE	(170*3)
#define TOTALBYTES	(96*48*3)
#define NUMUNIVERSES	((TOTALBYTES+(BYTES_PER_UNIVERSE-1)) / BYTES_PER_UNIVERSE)
#define BUFFERSIZE	1024

#define UNIVERSE_CONTROL	0xFFFF

int matrix_set_custom_baud(int fd, unsigned int baudrate) {
	struct termios2 t;
	if (ioctl(fd, TCGETS2, &t)==-1) {
		printf("Error on ioctl TCGETS2\r\n");
		return -1;
	}
	t.c_iflag = IGNBRK|IGNPAR;
	t.c_oflag = 0;
	t.c_cflag = BOTHER|CS8|CLOCAL;
	t.c_ospeed = t.c_ispeed = baudrate;
	//t.c_cflag &= ~CBAUD;
	//t.c_cflag |= BOTHER;
	if (ioctl(fd, TCSETS2, &t)) {
		printf("Error on ioctl TCGETS2\r\n");
		return -1;
	}
	printf("Sizeof termios2: %d\r\n",sizeof(t));
	printf("TCGETS2: %d 0x%.8X\r\n",TCGETS2,TCGETS2);
	printf("TCSETS2: %d 0x%.8X\r\n",TCSETS2,TCSETS2);
	return 0;
}

int matrix_fd;
int socket_fd;

int matrix_init() {
	matrix_fd = open("/dev/ttyATH0",O_RDWR);
	if (matrix_fd == -1) {
		printf("Unable to open /dev/ttyATH0\r\n");
		return -1;
	}
	if (matrix_set_custom_baud(matrix_fd, 5000000) == -1) {
		printf("Unable to set custom baudrate\r\n");
		return -1;
	}
	return 0;
}

void matrix_deinit() {
	close(matrix_fd);
}

void matrix_write(unsigned char *data, unsigned int len) {
	while (len) {
		unsigned int blocklen = MIN(1024, len);
		write(matrix_fd,data,blocklen);
		data = &data[blocklen];
		len -= blocklen;
	}
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

void matrix_swap() {
	static unsigned char swappacket[]={'F','W','C',0};
	matrix_write(swappacket,sizeof(swappacket));
}

void matrix_setgamma(unsigned int gamma) {
	unsigned char command[]={'F','W','C',(1 << 5) | 1,gamma};
	matrix_write(command, sizeof(command));
}

void matrix_setbrightness(unsigned int brightness) {
	unsigned char command[]={'F','W','C',(2 << 5) | 1,brightness};
	matrix_write(command, sizeof(command));
}

int handle_opoutput(unsigned char Sequence, unsigned char Physical, unsigned short Universe, unsigned short Length, unsigned char *data) {
	if (Universe < NUMUNIVERSES) {
		unsigned int offset = Universe * BYTES_PER_UNIVERSE;
		unsigned int len = Length;
		unsigned int packetlen = len + 7; //FWHLLOO
		//Abuse the fact that the 7 bytes before data are still part of the Art-Net packet, so we can write to that!
		unsigned char *packet = &data[-7];
		packet[0]='F';
		packet[1]='W';
		packet[2]='P';
		packet[3]=len>>8;
		packet[4]=len&0xFF;
		packet[5]=offset >> 8;
		packet[6]=offset & 0xFF;
		matrix_write(packet, packetlen);
		if (Universe == (NUMUNIVERSES-1))
			matrix_swap();
	}
	if (Universe == UNIVERSE_CONTROL) {
		if (Length > 0) matrix_setbrightness(data[0]);
		if (Length > 1) matrix_setgamma(data[1]);
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