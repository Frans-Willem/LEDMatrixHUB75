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

#define IMAGE_WIDTH		96
#define IMAGE_HEIGHT	48

#define IMAGE_LEN		(IMAGE_WIDTH*IMAGE_HEIGHT*3)
#define COMMAND_LEN		(IMAGE_LEN+5)
#define IMAGE_OFFSET		4

#define BUFFERSIZE	1024

unsigned char command[COMMAND_LEN];
unsigned char *image=&command[IMAGE_OFFSET];
unsigned int command_sent = 0;
unsigned int image_ready=0;

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
	
	command[0]=0xC9;
	command[1]=0xDA;
	command[2] = IMAGE_LEN >> 8;
	command[3] = IMAGE_LEN & 0xFF;
	command[COMMAND_LEN-1]=0x36;
	image_ready = 0;
	return 0;
}

void matrix_deinit() {
	close(matrix_fd);
}

void matrix_flush() {
	unsigned int command_ready = image_ready + IMAGE_OFFSET;
	if (command_ready == IMAGE_LEN + IMAGE_OFFSET)
		command_ready = COMMAND_LEN;
	while (command_ready > command_sent) {
		unsigned int blocklen = MIN(1024, command_ready-command_sent);
		write(matrix_fd,&command[command_sent], blocklen);
		command_sent += blocklen;
	}
	if (command_sent == COMMAND_LEN) {
		command_sent = 0;
		image_ready = 0;
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

int handle_opoutput(unsigned char Sequence, unsigned char Physical, unsigned short Universe, unsigned short Length, unsigned char *data) {
	unsigned int offset = Universe * BYTES_PER_UNIVERSE;
	unsigned int len = Length;
	if (offset > IMAGE_LEN) {
		return 0;
	}
	if (offset + len > IMAGE_LEN) len = IMAGE_LEN - offset;
	memcpy(&image[offset],data,len);
	image_ready = MAX(image_ready, offset+len);
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
			while (socket_handle() >= 0)
				matrix_flush();
			socket_deinit();
		}
		matrix_deinit();
	}
	return 0;
}