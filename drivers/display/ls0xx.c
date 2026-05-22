/**
 * @file ls0xx.c
 * @brief Custom driver for Sharp LS0xx series monochrome displays.
 *
 * This driver provides support for LS0xx series displays, including custom
 * 180-degree rotation specifically for the 1-bit-per-pixel (A1) format,
 * which is not natively supported by LVGL's software rotation.
 *
 * Copyright (c) 2020 Rohit Gujarathi
 * Copyright (c) 2024 The Gemini Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT   sharp_ls0xx

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ls0xx, CONFIG_DISPLAY_LOG_LEVEL);

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>

// LVGL include for rotation
#include <lvgl.h>

/* Supports LS012B7DD01, LS013B7DH03, LS013B7DH05
 * LS027B7DH01A, LS032B7DD02, LS044Q7DH01
 */

/**
 * @name LS0xx Display Configuration
 * @{
 * @brief Macros derived from Devicetree and display specifications.
 */

/** @brief Display panel width in pixels. */
#define LS0XX_PANEL_WIDTH   DT_INST_PROP(0, width)
/** @brief Display panel height in pixels. */
#define LS0XX_PANEL_HEIGHT  DT_INST_PROP(0, height)

/** @brief Pixels per byte for monochrome displays. */
#define LS0XX_PIXELS_PER_BYTE  8U

/**
 * @brief Total bytes per line sent over SPI.
 *
 * Each line transfer consists of:
 * - 8-bit line address/control
 * - Width-bits of pixel data
 * - 8-bit dummy/trailer
 */
#define LS0XX_BYTES_PER_LINE  ((LS0XX_PANEL_WIDTH / LS0XX_PIXELS_PER_BYTE) + 2)

/** @brief Command bit to write data to the display. */
#define LS0XX_BIT_WRITECMD    0x01
/** @brief Command bit to toggle VCOM (software-driven). */
#define LS0XX_BIT_VCOM        0x02
/** @brief Command bit to clear the display. */
#define LS0XX_BIT_CLEAR       0x04
/** @} */

/**
 * @brief Global buffer for rotated data.
 * @todo Move to a per-instance structure if multi-display support is needed.
 */
static uint8_t rotated_buf[LS0XX_PANEL_WIDTH / 8 * LS0XX_PANEL_HEIGHT];

/**
 * @brief Local buffer for incoming display data.
 * @todo Consider if memcpy and this buffer are strictly necessary, or if we can process in-place.
 */
static uint8_t local_buf[LS0XX_PANEL_WIDTH / 8 * LS0XX_PANEL_HEIGHT];

/**
 * @brief Driver configuration structure.
 *
 * Holds hardware-specific configuration like SPI bus and GPIO pins.
 */
struct ls0xx_config {
	/** @brief SPI device specification. */
	struct spi_dt_spec bus;
#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	/** @brief GPIO for display enable (optional). */
	struct gpio_dt_spec disp_en_gpio;
#endif
#if DT_INST_NODE_HAS_PROP(0, extcomin_gpios)
	/** @brief GPIO for external VCOM toggle (optional). */
	struct gpio_dt_spec extcomin_gpio;
#endif
};

#if DT_INST_NODE_HAS_PROP(0, extcomin_gpios)
/**
 * @brief Thread for toggling VCOM pin.
 *
 * Sharp LCDs require a periodic signal to prevent DC voltage buildup, which
 * can damage the panel. This thread toggles the EXTCOMIN pin at the frequency
 * specified in Devicetree.
 */
static void ls0xx_vcom_toggle(void *a, void *b, void *c)
{
	const struct ls0xx_config *config = a;

	while (1) {
		gpio_pin_toggle_dt(&config->extcomin_gpio);
		k_usleep(3);
		gpio_pin_toggle_dt(&config->extcomin_gpio);
		k_msleep(1000 / DT_INST_PROP(0, extcomin_frequency));
	}
}

K_THREAD_STACK_DEFINE(vcom_toggle_stack, 512);
struct k_thread vcom_toggle_thread;
#endif

/**
 * @brief Logs the start of a buffer in hexadecimal format.
 *
 * @param prefix Tag to display before the buffer content.
 * @param buf Pointer to the buffer.
 * @param len Total length of the buffer.
 */
static void log_buf_head(const char *prefix, const uint8_t *buf, size_t len)
{
#ifdef DISPLAY_DRV_DEBUG
    if (buf == NULL || len == 0) {
        LOG_INF("%s: NULL buffer or zero length", prefix);
        return;
    }
    size_t display_len = len > 8 ? 8 : len;
    char log_str[40] = {0};
    for(size_t i = 0; i < display_len; i++) {
        sprintf(log_str + i*3, "%02x ", buf[i]);
    }
    LOG_INF("%s: %s", prefix, log_str);
#endif
}


/**
 * @brief Turns display blanking off (enables display).
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code otherwise.
 */
static int ls0xx_blanking_off(const struct device *dev)
{
#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	const struct ls0xx_config *config = dev->config;

	return gpio_pin_set_dt(&config->disp_en_gpio, 1);
#else
	LOG_WRN("Unsupported: no disp-en-gpios");
	return -ENOTSUP;
#endif
}

/**
 * @brief Turns display blanking on (disables display).
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code otherwise.
 */
static int ls0xx_blanking_on(const struct device *dev)
{
#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	const struct ls0xx_config *config = dev->config;

	return gpio_pin_set_dt(&config->disp_en_gpio, 0);
#else
	LOG_WRN("Unsupported: no disp-en-gpios");
	return -ENOTSUP;
#endif
}

/**
 * @brief Sends a command to the display over SPI.
 *
 * @param dev Pointer to the device structure.
 * @param buf Pointer to the command buffer.
 * @param len Length of the command.
 * @return Result of the SPI write operation.
 */
static int ls0xx_cmd(const struct device *dev, uint8_t *buf, uint8_t len)
{
	const struct ls0xx_config *config = dev->config;
	struct spi_buf cmd_buf = { .buf = buf, .len = len };
	struct spi_buf_set buf_set = { .buffers = &cmd_buf, .count = 1 };

	return spi_write_dt(&config->bus, &buf_set);
}

/**
 * @brief Clears the entire display.
 *
 * Sends the clear command which sets all pixels to white (default state).
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code otherwise.
 */
static int ls0xx_clear(const struct device *dev)
{
	const struct ls0xx_config *config = dev->config;
	uint8_t clear_cmd[2] = { LS0XX_BIT_CLEAR, 0 };
	int err;

#ifdef DISPLAY_DRV_DEBUG
	LOG_INF("Clearing display");
#endif
	err = ls0xx_cmd(dev, clear_cmd, sizeof(clear_cmd));
	spi_release_dt(&config->bus);

	return err;
}

/**
 * @brief Internal function to update a range of lines on the display.
 *
 * This function handles the Sharp-specific SPI protocol for updating multiple lines.
 * Each line consists of a line address, the pixel data, and trailing dummy bits.
 *
 * @param dev Pointer to the device structure.
 * @param start_line The first line to update (1-based index).
 * @param num_lines Number of consecutive lines to update.
 * @param data Pointer to the pixel data for the lines.
 * @return 0 on success, negative error code otherwise.
 */
static int ls0xx_update_display(const struct device *dev,
				uint16_t start_line,
				uint16_t num_lines,
				const uint8_t *data)
{
	const struct ls0xx_config *config = dev->config;
	uint8_t write_cmd[1] = { LS0XX_BIT_WRITECMD };
	uint16_t ln = start_line;
	uint8_t addr_0_1 = 0;
	uint8_t addr_2_9 = (ln >> 2);
	struct spi_buf line_buf[3] = {
		{
			.len = sizeof(addr_2_9),
			.buf = &addr_2_9,
		},
		{
			.len = LS0XX_BYTES_PER_LINE - 2,
		},
		{
			.len = sizeof(addr_0_1),
			.buf = &addr_0_1,
		},
	};
	struct spi_buf_set line_set = {
		.buffers = line_buf,
		.count = ARRAY_SIZE(line_buf),
	};
	int err;

#ifdef DISPLAY_DRV_DEBUG
	LOG_DBG("Updating lines %d to %d", start_line, start_line + num_lines - 1);
#endif

	/* The Sharp protocol expects the command and the first part of the address
	 * in the first byte.
	 */
	write_cmd[0] = ( (LS0XX_BIT_WRITECMD) |
	                 ((ln & 0x03) << 6)   
                   );					 
	err = ls0xx_cmd(dev, write_cmd, sizeof(write_cmd));

	/* Send each line to the screen including
	 * the line number and dummy bits.
	 * The address is split across two bytes in a specific way for this controller.
	 */
	for (; ln <= start_line + num_lines - 1; ln++) {
		addr_2_9 = (ln >> 2);
		line_buf[1].buf = (uint8_t *)data;
		/* Prepare the address for the NEXT line's prefix/trailer */
		addr_0_1 = (((ln + 1) & 0x03) << 6);
		err |= spi_write_dt(&config->bus, &line_set);
		data += LS0XX_PANEL_WIDTH / LS0XX_PIXELS_PER_BYTE;
	}

	/* Send another trailing 8 bits for the last line to complete the transfer.
	 */
	err |= ls0xx_cmd(dev, write_cmd, sizeof(write_cmd));

	spi_release_dt(&config->bus);

	return err;
}

/**
 * @brief Rotates a 1-bit-per-pixel buffer 180 degrees.
 *
 * This function is a custom replacement for LVGL's software rotation,
 * which does not currently support the A1 monochrome format.
 * It performs a 180-degree rotation by reversing:
 * 1. The order of rows.
 * 2. The order of bytes within each row.
 * 3. The order of bits within each byte.
 *
 * @param src Pointer to the source buffer.
 * @param dst Pointer to the destination (rotated) buffer.
 * @param width Width of the image in pixels.
 * @param height Height of the image in pixels.
 */
static void rotate180_a1(const uint8_t *src, uint8_t *dst, int32_t width, int32_t height)
{
	int32_t line_bytes = width / 8;
        const uint8_t *src_line = src;
        uint8_t *dst_line = dst + (height - 1) * line_bytes;
	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 0; x < line_bytes; x++) {
		        dst_line[line_bytes - 1 - x] = ((uint8_t)__RBIT(src_line[x]) >> 24);
		}
        src_line += line_bytes;
		dst_line -= line_bytes;
	}
}

/**
 * @brief Writes a buffer to the display.
 *
 * This is the main entry point for updating the display from LVGL.
 * It handles optional 180-degree rotation before updating the hardware.
 *
 * @param dev Pointer to the device structure.
 * @param x Horizontal start position (must be 0 for this driver).
 * @param y Vertical start position.
 * @param desc Descriptor of the display buffer.
 * @param buf Pointer to the pixel data.
 * @return 0 on success, negative error code otherwise.
 */
static int ls0xx_write(const struct device *dev, const uint16_t x,
		       const uint16_t y,
		       const struct display_buffer_descriptor *desc,
		       const void *buf)
{
	if (buf == NULL) {
		LOG_WRN("Display buffer is not available");
		return -EINVAL;
	}

    size_t buf_size = desc->width * desc->height / 8;
    memcpy(local_buf, buf, buf_size);

    /* Get current rotation from LVGL.
     * @todo Check if we should use the display-specific rotation if available.
     */
    lv_disp_t *disp = lv_disp_get_default();
    lv_disp_rotation_t rot = lv_disp_get_rotation(disp);

    if (rot == LV_DISP_ROTATION_180) {
#ifdef DISPLAY_DRV_DEBUG
        LOG_INF("ROTATING: x=%d, y=%d, w=%d, h=%d, pitch=%d", x, y, desc->width, desc->height, desc->pitch);
#endif
        log_buf_head("INPUT BUF", local_buf, buf_size);

        /* Custom rotation for A1 format */
        rotate180_a1(local_buf, rotated_buf, desc->width, desc->height);
        log_buf_head("ROTATED BUF", rotated_buf, buf_size);

        /* Calculate new Y position for 180-degree rotation.
         * The coordinate system is flipped vertically.
         */
        uint16_t new_y = LS0XX_PANEL_HEIGHT - y - desc->height;

#ifdef DISPLAY_DRV_DEBUG
        LOG_INF("ROTATED TO: new_y=%d", new_y);
#endif

        /* Adding 1 because Sharp display line numbering starts at 1 */
        ls0xx_update_display(dev, new_y + 1, desc->height, rotated_buf);
    } else {
#ifdef DISPLAY_DRV_DEBUG
        LOG_INF("NOT ROTATING: x=%d, y=%d, w=%d, h=%d", x, y, desc->width, desc->height);
#endif
        /* Standard update without rotation */
	    ls0xx_update_display(dev, y + 1, desc->height, local_buf);
    }

    lv_disp_flush_ready(disp);

	return 0;
}

/**
 * @brief Retrieves the capabilities of the display.
 *
 * @param dev Pointer to the device structure.
 * @param caps Pointer to the display_capabilities structure to fill.
 */
static void ls0xx_get_capabilities(const struct device *dev,
				   struct display_capabilities *caps)
{
	memset(caps, 0, sizeof(struct display_capabilities));
	caps->x_resolution = LS0XX_PANEL_WIDTH;
	caps->y_resolution = LS0XX_PANEL_HEIGHT;
	caps->supported_pixel_formats = PIXEL_FORMAT_MONO01;
	caps->current_pixel_format = PIXEL_FORMAT_MONO01;
	caps->screen_info = SCREEN_INFO_X_ALIGNMENT_WIDTH;
}

/**
 * @brief Sets the pixel format for the display.
 *
 * Currently only MONO01 is supported.
 *
 * @param dev Pointer to the device structure.
 * @param pf Desired pixel format.
 * @return 0 if format is supported, negative error code otherwise.
 */
static int ls0xx_set_pixel_format(const struct device *dev,
				  const enum display_pixel_format pf)
{
	if (pf == PIXEL_FORMAT_MONO01) {
		return 0;
	}

	LOG_ERR("Pixel format not supported");
	return -ENOTSUP;
}

/** @brief Static configuration for the display instance. */
static const struct ls0xx_config ls0xx_config = {
	.bus = SPI_DT_SPEC_INST_GET(
		0, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
		SPI_TRANSFER_LSB | SPI_CS_ACTIVE_HIGH |
		SPI_HOLD_ON_CS | SPI_LOCK_ON, 0),
#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	.disp_en_gpio = GPIO_DT_SPEC_INST_GET(0, disp_en_gpios),
#endif
#if DT_INST_NODE_HAS_PROP(0, extcomin_gpios)
	.extcomin_gpio = GPIO_DT_SPEC_INST_GET(0, extcomin_gpios),
#endif
};

/**
 * @brief Initializes the LS0xx display driver.
 *
 * Checks hardware readiness, configures GPIOs, starts VCOM thread,
 * and clears the display.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code otherwise.
 */
static int ls0xx_init(const struct device *dev)
{
	const struct ls0xx_config *config = &ls0xx_config;

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	if (!gpio_is_ready_dt(&config->disp_en_gpio)) {
		LOG_ERR("DISP port device not ready");
		return -ENODEV;
	}
#ifdef DISPLAY_DRV_DEBUG
	LOG_INF("Configuring DISP pin to OUTPUT_HIGH");
#endif
	gpio_pin_configure_dt(&config->disp_en_gpio, GPIO_OUTPUT_HIGH);
#endif

#if DT_INST_NODE_HAS_PROP(0, extcomin_gpios)
	if (!gpio_is_ready_dt(&config->extcomin_gpio)) {
		LOG_ERR("EXTCOMIN port device not ready");
		return -ENODEV;
	}
#ifdef DISPLAY_DRV_DEBUG
	LOG_INF("Configuring EXTCOMIN pin");
#endif
	gpio_pin_configure_dt(&config->extcomin_gpio, GPIO_OUTPUT_LOW);

	/* Start thread for toggling VCOM */
	k_tid_t vcom_toggle_tid = k_thread_create(&vcom_toggle_thread,
						  vcom_toggle_stack,
						  K_THREAD_STACK_SIZEOF(vcom_toggle_stack),
						  ls0xx_vcom_toggle,
						  (void *)config, NULL, NULL,
						  3, 0, K_MSEC(100));
	k_thread_name_set(vcom_toggle_tid, "ls0xx_vcom");
#endif  /* DT_INST_NODE_HAS_PROP(0, extcomin_gpios) */

	LOG_INF("LS0xx initialized");
	/* Clear display else it shows random data */
	return ls0xx_clear(dev);
}

/** @brief Display driver API implementation. */
static const struct display_driver_api ls0xx_driver_api = {
	.blanking_on = ls0xx_blanking_on,
	.blanking_off = ls0xx_blanking_off,
	.write = ls0xx_write,
	.get_capabilities = ls0xx_get_capabilities,
	.set_pixel_format = ls0xx_set_pixel_format,
};

/* Define the display device instance */
DEVICE_DT_INST_DEFINE(0, ls0xx_init, NULL, NULL, &ls0xx_config, POST_KERNEL,
		      CONFIG_DISPLAY_INIT_PRIORITY, &ls0xx_driver_api);
