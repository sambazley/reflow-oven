#include "common.h"
#include <errno.h>
#include <libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static libusb_device_handle *oven = 0;
static int libusb_inited = 0;
static int iface_claimed = 0;

static int on_fault(struct usb_packet_fault *fault)
{
	switch (fault->fault) {
	case THERMO_NO_FAULT:
		fprintf(stderr, "No fault?\n");
		return -1;
	case THERMO_UNKNOWN:
		fprintf(stderr, "Unknown error\n");
		return -1;
	case THERMO_OPEN_CIRCUIT:
		fprintf(stderr, "Open circuit\n");
		return -1;
	case THERMO_SHORT_TO_VCC:
		fprintf(stderr, "Short to VCC\n");
		return -1;
	case THERMO_SHORT_TO_GND:
		fprintf(stderr, "Short to GND\n");
		return -1;
	case THERMO_D3:
		fprintf(stderr, "Malformed SPI data (D3)\n");
		return -1;
	case THERMO_D17:
		fprintf(stderr, "Malformed SPI data (D17)\n");
		return -1;
	default:
		fprintf(stderr, "Unknown fault?\n");
		return -1;
	}
}

static int receive_data(libusb_device_handle *dev, uint8_t **dest, size_t *len)
{
	int r;

	*dest = 0;
	*len = 0;

	while (1) {
		unsigned char buf [8];
		int rlen;
		r = libusb_bulk_transfer(dev, 0x82, buf, sizeof(buf), &rlen, 0);
		if (r < 0) {
			fprintf(stderr, "Failed to receive data %d\n", r);
			goto end;
		}

		*len += rlen;
		*dest = realloc(*dest, *len);
		memcpy(*dest + *len - rlen, buf, rlen);

		uint8_t packet_length = ((uint8_t *) *dest)[0];

		if (*len >= sizeof(struct usb_packet_fault)) {
			struct usb_packet_fault *fault =
				(struct usb_packet_fault *) *dest;
			if (fault->type == USB_PACKET_FAULT) {
				r = on_fault(fault);
				break;
			}
		}

		if (*len > packet_length) {
			r = -E2BIG;
			break;
		} else if (*len == packet_length) {
			r = 0;
			break;
		}
	}

end:
	return r;
}

static int send_data(libusb_device_handle *dev, union usb_packet_out *packet)
{
	int transfered = 0;
	int r;

	uint8_t packet_length = ((uint8_t *) packet)[0];

	while (1) {
		unsigned char *src = ((unsigned char *) packet) + transfered;
		r = libusb_bulk_transfer(dev, 0x01, src, packet->any.length, &transfered, 0);
		if (r < 0) {
			goto end;
		}

		if (transfered > packet->start.length) {
			r = -E2BIG;
			break;
		} else if (transfered == packet_length) {
			r = 0;
			break;
		}
	}

end:
	return r;
}

static void close_dev()
{
	if (iface_claimed) {
		libusb_release_interface(oven, 0);
		iface_claimed = 0;
	}

	if (oven) {
		libusb_close(oven);
	}

	if (libusb_inited) {
		libusb_exit(NULL);
		libusb_inited = 0;
	}
}

static int open_dev()
{
	int r;
	ssize_t dev_cnt;
	libusb_device **devs;

	if (libusb_inited) {
		fprintf(stderr, "libusb already initialised\n");
		return 1;
	}

	r = libusb_init(NULL);

	if (r < 0) {
		fprintf(stderr, "Failed to initialize libusb\n");
		return r;
	}
	libusb_inited = 1;

	r = dev_cnt = libusb_get_device_list(NULL, &devs);
	if (r < 0) {
		fprintf(stderr, "Failed to get device list\n");
		return r;
	}

	for (int i = 0; i < dev_cnt; i++) {
		struct libusb_device_descriptor desc;
		libusb_device *dev = devs[i];

		r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0) {
			fprintf(stderr, "Cannot get device descriptor\n");
			break;
		}

		if (desc.idVendor == 0x9876 && desc.idProduct == 0x6789) {
			r = libusb_open(dev, &oven);
			if (r < 0) {
				fprintf(stderr, "Failed to open device\n");
			}
			break;
		}
	}

	libusb_free_device_list(devs, 1);

	if (r < 0) {
		return r;
	}

	if (!oven) {
		fprintf(stderr, "No device found\n");
		r = 1;
		goto exit;
	}

	r = libusb_claim_interface(oven, 0);
	if (r < 0) {
		fprintf(stderr, "Failed to claim interface\n");
		goto exit;
	}
	iface_claimed = 1;

	return 0;
exit:
	close_dev();
	return r;
}

static int start(int argc, char *argv[])
{
	int r;

	(void) argc;
	(void) argv;

	if ((r = open_dev())) {
		return r;
	}

	union usb_packet_out packet;

	packet.start.length = sizeof(struct usb_packet_start);
	packet.start.type = USB_PACKET_START;

	r = send_data(oven, &packet);
	if (r < 0) {
		fprintf(stderr, "Failed to send command %d\n", r);
		goto exit;
	}

	while (1) {
		uint8_t *rbuf;
		size_t len;
		r = receive_data(oven, &rbuf, &len);

		if (r < 0) {
			if (rbuf) {
				free(rbuf);
			}
			break;
		}

		r = libusb_bulk_transfer(oven, 0x01, 0, 0, 0, 0);

		if (r < 0) {
			if (rbuf) {
				free(rbuf);
			}
			break;
		}

		struct usb_packet_temp *data = (struct usb_packet_temp *) rbuf;

		if (data->temp == 0 && data->target == 0) {
			free(rbuf);
			break;
		}

		printf("%.2f %.2f\n", data->temp, data->target);
		fflush(stdout);
		free(rbuf);
	}

exit:
	close_dev();

	return r;
}

int query(int argc, char *argv[]) {
	int r;

	(void) argc;
	(void) argv;

	if ((r = open_dev())) {
		return r;
	}

	union usb_packet_out out;

	out.start.length = sizeof(struct usb_packet_query_temp);
	out.start.type = USB_PACKET_QUERY_TEMP;

	r = send_data(oven, &out);
	if (r < 0) {
		fprintf(stderr, "Failed to send command %d\n", r);
		goto exit;
	}

	uint8_t *rbuf;
	size_t len;
	r = receive_data(oven, &rbuf, &len);

	if (r < 0) {
		if (rbuf) {
			free(rbuf);
		}
		goto exit;
	}

	struct usb_packet_temp *data = (struct usb_packet_temp *) rbuf;

	printf("%.2f\n", data->temp);

exit:
	close_dev();

	return r;
}

static struct cmd {
	const char *cmd;
	const char *desc;
	int (*func)(int argc, char *argv[]);
} cmds [] = {
	{"start", "Start the oven and print temperature values", start},
	{"query", "Queries the current temperature", query}
};

#define CMD_COUNT (sizeof(cmds) / sizeof(struct cmd))

static void print_usage(char *argv)
{
	fprintf(stderr, "Usage: %s [command]\n\n", argv);
	fprintf(stderr, "Commands:\n");

	for (size_t i = 0; i < CMD_COUNT; i++) {
		fprintf(stderr, "  %s -- %s\n", cmds[i].cmd, cmds[i].desc);
	}
}

int main(int argc, char *argv[])
{
	if (argc == 1) {
		print_usage(argv[0]);
		return 1;
	}

	for (size_t i = 0; i < CMD_COUNT; i++) {
		if (strcmp(argv[1], cmds[i].cmd) == 0) {
			return cmds[i].func(argc, argv);
		}
	}

	print_usage(argv[0]);

	return 1;
}
