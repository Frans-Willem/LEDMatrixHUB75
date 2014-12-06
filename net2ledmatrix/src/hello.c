#include <stdio.h>
#include <sys/types.h>
#include <libusb.h>

int main(void)
{
	libusb_device **devs;
	int r;
	ssize_t cnt;

	r = libusb_init(NULL);
	if (r < 0) {
		printf("libusb_init failed\r\n");
		return r;
	}

	libusb_device_handle *handle = libusb_open_device_with_vid_pid(NULL, 5824, 1500);
	if (handle == NULL) {
		printf("Unable to open device\r\n");
	} else {
		if (libusb_claim_interface(handle, 0) != 0) {
			printf("Unable to claim interface\r\n");
		} else {
			libusb_control_transfer(handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE, 0, 0, 0, NULL, 0, 1000);
			libusb_release_interface(handle, 0);
		}
		libusb_close(handle);
	}

	libusb_exit(NULL);
	return 0;
}