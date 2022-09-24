/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2021-2022 Jean THOMAS <virgule@jeanthomas.me>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <libusb.h>
#include "programmer.h"

struct dirtyjtag_spi_data {
	struct libusb_device_handle *libusb_handle;
	enum {
		DJTAG1,
		DJTAG2,
		DJTAGUNK,
	} protocol_version;
};

static const struct dev_entry devs_dirtyjtag_spi[] = {
	{ 0x1209, 0xC0CA, OK, "DirtyJTAG", "JTAG probe" },
	{ 0 },
};

static const char dirtyjtag_write_endpoint = 0x01;
static const char dirtyjtag_read_endpoint = 0x82;
static const int dirtyjtag_timeout = 100 * 10; /* 100 ms */

enum dirytjtag_command_identifier {
  CMD_STOP = 0x00,
  CMD_INFO = 0x01,
  CMD_FREQ = 0x02,
  CMD_XFER = 0x03,
  CMD_SETSIG = 0x04,
  CMD_GETSIG = 0x05,
  CMD_CLK = 0x06
};

enum dirytjtag_signal_identifier {
  SIG_TCK = 1 << 1,
  SIG_TDI = 1 << 2,
  SIG_TDO = 1 << 3,
  SIG_TMS = 1 << 4,
  SIG_TRST = 1 << 5,
  SIG_SRST = 1 << 6
};

static int dirtyjtag_send(struct dirtyjtag_spi_data *djtag_data, const char* data, size_t len)
{
	int transferred;
	int ret = libusb_bulk_transfer(djtag_data->libusb_handle,
		dirtyjtag_write_endpoint,
		(unsigned char*)data,
		len,
		&transferred,
		dirtyjtag_timeout);
	if (ret != 0) {
		msg_perr("%s: failed to send query command\n", __func__);
		return -1;
	}
	if (transferred != (int)len) {
		msg_perr("%s: failed to send whole packet\n", __func__);
		return -1;
	}

	return 0;
}

static int dirtyjtag_receive(struct dirtyjtag_spi_data *djtag_data, char* data, size_t buffer_len, int expected)
{
	int transferred;
	int ret = libusb_bulk_transfer(djtag_data->libusb_handle,
		dirtyjtag_read_endpoint,
		(unsigned char*)data,
		buffer_len,
		&transferred,
		dirtyjtag_timeout);
	if (ret != 0) {
		msg_perr("%s: Failed to read SPI commands\n", __func__);
		return -1;
	}

	if (expected != -1 && transferred != expected) {
		msg_perr("%s: failed to meet expected\n", __func__);
		return -1;
	}

	return transferred;
}

static int dirtyjtag_spi_shutdown(void *data)
{
	struct dirtyjtag_spi_data *djtag_data = (struct dirtyjtag_spi_data*)data;
	libusb_close(djtag_data->libusb_handle);
	free(data);
	return 0;
}

static inline unsigned int _min(unsigned int a, unsigned int b)
{
	return (a < b) ? a : b;
}

static int dirtyjtag_djtag1_spi_send_command(struct dirtyjtag_spi_data *context, unsigned int writecnt, unsigned int readcnt, const unsigned char *writearr, unsigned char *readarr)
{
	const size_t max_xfer_size = 30; // max transfer size in DJTAG1
	size_t len = writecnt+readcnt;
	size_t num_xfer = (len+max_xfer_size-1)/max_xfer_size; // ceil(len/max_xfer_size)

	char *tx_buf = malloc(max_xfer_size*num_xfer);
	if (!tx_buf) {
		msg_perr("%s: Failed tx_buf allocation", __func__);
		return -1;
	}
	char *rx_buf = malloc(max_xfer_size*num_xfer);
	if (!rx_buf) {
		msg_perr("%s: Failed rx_buf allocation", __func__);
		return -1;
	}

	memcpy(tx_buf, writearr, writecnt);
	for (size_t i = 0; i < num_xfer; i++) {
		uint8_t readout_buffer[32], command_buffer[32] = {CMD_XFER};
		size_t txn_size;
		if (i == num_xfer-1 && len%max_xfer_size != 0) {
			txn_size = len%max_xfer_size;
		} else {
			txn_size = 30;
		}
		command_buffer[1] = txn_size*8;
		memcpy(command_buffer+2, tx_buf+30*i, txn_size);
		dirtyjtag_send(context, (const char*)command_buffer, sizeof(command_buffer));
		dirtyjtag_receive(context, (char*)readout_buffer, sizeof(readout_buffer), 32);
		memcpy(rx_buf+i*30, readout_buffer, txn_size);
	}
	memcpy(readarr, rx_buf+writecnt, readcnt);

	free(tx_buf);
	free(rx_buf);

	const uint8_t tms_reset_buffer[] = {
		CMD_SETSIG,
		SIG_TMS,
		SIG_TMS,

		CMD_STOP,
	};
	dirtyjtag_send(context, (const char*)tms_reset_buffer, sizeof(tms_reset_buffer));

	return 0;
}

static int dirtyjtag_spi_spi_send_command(const struct flashctx *flash, unsigned int writecnt, unsigned int readcnt, const unsigned char *writearr, unsigned char *readarr)
{
	struct dirtyjtag_spi_data *djtag_data = flash->mst->spi.data;
	return dirtyjtag_djtag1_spi_send_command(djtag_data, writecnt, readcnt, writearr, readarr);
}

static const struct spi_master spi_master_dirtyjtag_spi = {
	.features	= SPI_MASTER_4BA,
	.max_data_read	= 30,
	.max_data_write	= 30,
	.command	= dirtyjtag_spi_spi_send_command,
	.multicommand	= default_spi_send_multicommand,
	.read		= default_spi_read,
	.write_256	= default_spi_write_256,
	.write_aai	= default_spi_write_aai,
	.shutdown	= dirtyjtag_spi_shutdown,
};

static int dirtyjtag_spi_init(const struct programmer_cfg *cfg)
{
	struct libusb_device_handle *handle = NULL;
	struct dirtyjtag_spi_data *djtag_data = NULL;

	int ret = libusb_init(NULL);
	if (ret < 0) {
		msg_perr("%s: couldn't initialize libusb!\n", __func__);
		return -1;
	}

#if LIBUSB_API_VERSION < 0x01000106
	libusb_set_debug(NULL, 3);
#else
	libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
#endif

	uint16_t vid = devs_dirtyjtag_spi[0].vendor_id;
	uint16_t pid = devs_dirtyjtag_spi[0].device_id;
	handle = libusb_open_device_with_vid_pid(NULL, vid, pid);
	if (handle == NULL) {
		msg_perr("%s: couldn't open device %04x:%04x.\n", __func__, vid, pid);
		return -1;
	}

	ret = libusb_set_auto_detach_kernel_driver(handle, 1);
	if (ret != 0) {
		msg_pwarn("Platform does not support detaching of USB kernel drivers.\n"
			  "If an unsupported driver is active, claiming the interface may fail.\n");
	}

	ret = libusb_claim_interface(handle, 0);
	if (ret != 0) {
		msg_perr("%s: failed to claim interface 0: '%s'\n", __func__, libusb_error_name(ret));
		goto cleanup;
	}

	djtag_data = calloc(1, sizeof(struct dirtyjtag_spi_data));
	if (djtag_data == NULL) {
		msg_perr("%s: failed to allocate internal driver data structure\n", __func__);
		goto cleanup;
	}

	djtag_data->libusb_handle = handle;

	unsigned long int freq = 100;
	char *tmp = extract_programmer_param_str(cfg, "frequency");
	if (tmp) {
		char *units = tmp;
		char *end = tmp + strlen(tmp);

		errno = 0;
		freq = strtoul(tmp, &units, 0);
		if (errno) {
			msg_perr("Invalid frequency \"%s\", %s\n",
					tmp, strerror(errno));
			free(tmp);
			return 1;
		}

		if ((units > tmp) && (units < end)) {
			bool units_valid = false;

			if (units < end - 3) {
				;
			} else if (units == end - 2) {
				if (!strcasecmp(units, "hz"))
					units_valid = true;
			} else if (units == end - 3) {
				if (!strcasecmp(units, "khz")) {
					freq *= 1000;
					units_valid = true;
				} else if (!strcasecmp(units, "mhz")) {
					freq *= 1000000;
					units_valid = true;
				}
			}

			if (!units_valid) {
				msg_perr("Invalid units: %s\n", units);
				free(tmp);
				return 1;
			}
		}

		if (freq == 0) {
			msg_perr("%s: invalid value 0 for freq parameter\n", __func__);
			free(tmp);
			return 1;
		}

		if (freq < 1000) {
			msg_perr("%s: invalid value (lower than 1kHz) for freq parameter\n", __func__);
			free(tmp);
			return 1;
		}

		if (freq > 1000*65535) {
			msg_perr("%s: invalid value () for freq parameter\n", __func__);
			free(tmp);
			return 1;
		}

		freq /= 1000;
	}
	free(tmp);

	uint8_t commands[] = {
		CMD_SETSIG, /* Set all signals to low (except TRST - HOLD#) */
		SIG_TDI | SIG_TMS | SIG_TCK | SIG_SRST | SIG_TRST,
		SIG_SRST | SIG_TRST | SIG_TMS,

		CMD_FREQ, /* Set frequency */
		(freq >> 8) & 0xFF,
		freq & 0xFF,

		CMD_STOP,
	};
	ret = dirtyjtag_send(djtag_data, (const char*)commands, sizeof(commands));
	if (ret != 0) {
		msg_perr("%s: failed to configure DirtyJTAG into initialization state\n", __func__);
		goto cleanup;
	}

	return register_spi_master(&spi_master_dirtyjtag_spi, (void*)djtag_data);

cleanup:
	libusb_close(handle);
	free(djtag_data);
	return -1;
}

const struct programmer_entry programmer_dirtyjtag_spi = {
	.name			= "dirtyjtag_spi",
	.type			= USB,
	.devs.dev		= devs_dirtyjtag_spi,
	.init			= dirtyjtag_spi_init,
	.map_flash_region	= fallback_map,
	.unmap_flash_region	= fallback_unmap,
	.delay			= internal_delay,
};
