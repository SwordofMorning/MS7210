#include "ms7210_wrap.h"

static int ms7210_write(ms7210_dev_t *dev, uint16_t reg, uint8_t value)
{
    uint8_t buf[3];
    struct i2c_msg messages[1];
    struct i2c_rdwr_ioctl_data packets;

    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF; 
    buf[2] = value;

    messages[0].addr = MS7210_I2C_ADDR;
    messages[0].flags = 0;
    messages[0].len = 3;
    messages[0].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if(ioctl(dev->i2c_fd, I2C_RDWR, &packets) < 0) {
        printf("Write reg 0x%04x failed: %s\n", reg, strerror(errno));
        return -1;
    }

    return 0;
}

static int ms7210_read(ms7210_dev_t *dev, uint16_t reg, uint8_t *value)
{
    uint8_t buf[2];
    struct i2c_msg messages[2];
    struct i2c_rdwr_ioctl_data packets;

    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;

    messages[0].addr = MS7210_I2C_ADDR;
    messages[0].flags = 0;
    messages[0].len = 2;
    messages[0].buf = buf;

    messages[1].addr = MS7210_I2C_ADDR;
    messages[1].flags = I2C_M_RD;
    messages[1].len = 1;
    messages[1].buf = value;

    packets.msgs = messages;
    packets.nmsgs = 2;

    if(ioctl(dev->i2c_fd, I2C_RDWR, &packets) < 0) {
        printf("Read reg 0x%04x failed: %s\n", reg, strerror(errno));
        return -1;
    }

    return 0;
}

static int ms7210_update_bits(ms7210_dev_t *dev, uint16_t reg, 
                            uint8_t mask, uint8_t value)
{
    uint8_t tmp;
    int ret;

    ret = ms7210_read(dev, reg, &tmp);
    if(ret < 0)
        return ret;

    tmp &= ~mask;
    tmp |= value & mask;

    return ms7210_write(dev, reg, tmp);
}

static void ms7210_rc_freq_set(ms7210_dev_t *dev)
{
    ms7210_write(dev, MS7210_RC_CTRL1_REG, 0x81);
    ms7210_write(dev, MS7210_RC_CTRL1_REG, 0x54);
}

static void ms7210_csc_config_input(ms7210_dev_t *dev)
{
    ms7210_update_bits(dev, MS7210_CSC_CTRL1_REG, 0x03, 0x10 | dev->cs_mode);
}

static void ms7210_dig_pads_pull_set(ms7210_dev_t *dev, unsigned int pull)
{
    ms7210_update_bits(dev, MS7210_IO_CTRL3_REG, 0x30, (pull & 3) << 4);
}

static void ms7210_misc_audio_pad_in_spdif(ms7210_dev_t *dev)
{
    if(dev->spdif == 2) {
        ms7210_update_bits(dev, MS7210_RX_PLL_SEL_REG, 0x0c, 0x0c);
        ms7210_update_bits(dev, MS7210_AUPLL_CTRL2_REG, 0x10, 0x10);
        ms7210_update_bits(dev, MS7210_CLK_CTRL1_REG, 0x01, 0x01);
        
        ms7210_update_bits(dev, MS7210_AUPLL_PWR_REG, 0x02, 0x00);
        ms7210_update_bits(dev, MS7210_AUPLL_PWR_REG, 0x01, 0x01);
        ms7210_update_bits(dev, MS7210_AUPLL_M_REG, 0xff, 0x1c);
        ms7210_update_bits(dev, MS7210_AUPLL_CFG_CTRL_REG, 0xc0, 0x40);
        ms7210_update_bits(dev, MS7210_AUPLL_PWR_REG, 0x01, 0x00);

        usleep(100);

        ms7210_update_bits(dev, MS7210_AUPLL_PWR_REG, 0x02, 0x02);
        ms7210_update_bits(dev, MS7210_CLK_CTRL5_REG, 0x03, 0x03);
    } else {
        ms7210_update_bits(dev, MS7210_CLK_CTRL5_REG, 0x03, 0x00);
        if(dev->spdif)
            ms7210_update_bits(dev, MS7210_CLK_CTRL5_REG, 0x04, 0x04);
    }
    ms7210_update_bits(dev, MS7210_PINMUX_REG, 0x18, dev->spdif << 3);
}

int ms7210_init(ms7210_dev_t *dev)
{
    uint8_t chip_id;
    
    // Read chip ID
    if(ms7210_read(dev, MS7210_CHIPID0_REG, &chip_id) < 0)
        return -1;

    printf("MS7210 Chip ID: 0x%02x\n", chip_id);

    // Initialize device
    dev->cs_mode = DVIN_CS_MODE_RGB;
    dev->bw_mode = DVIN_BW_MODE_16_20_24BIT;
    dev->sq_mode = DVIN_SQ_MODE_NONSEQ;
    dev->dr_mode = DVIN_DR_MODE_SDR;
    dev->sy_mode = DVIN_SY_MODE_HSVSDE;
    dev->rgb = true;
    dev->spdif = 0;

    // Configure device
    ms7210_rc_freq_set(dev);
    ms7210_csc_config_input(dev);
    ms7210_dig_pads_pull_set(dev, 2);
    ms7210_misc_audio_pad_in_spdif(dev);

    return 0;
}

void print_usage(const char *name)
{
    printf("Usage: %s [options]\n", name);
    printf("Options:\n");
    printf("  -d, --device <dev>     I2C device path (default: /dev/i2c-1)\n");
    printf("  -w, --width <width>    Display width (default: 1920)\n");
    printf("  -h, --height <height>  Display height (default: 1080)\n");
    printf("  -r, --refresh <rate>   Refresh rate (default: 60)\n");
    printf("  -c, --color <mode>     Color space (rgb|yuv444|yuv422)\n");
    printf("  -s, --spdif <mode>     SPDIF mode (0:i2s, 1:spdif+mclk, 2:spdif)\n");
    printf("  -?, --help            Show this help\n");
}

static int ms7210_write16(ms7210_dev_t *dev, uint16_t reg, uint16_t value)
{
    int err;
    
    if ((err = ms7210_write(dev, reg, value & 0xFF)) < 0)
        return err;
        
    return ms7210_write(dev, reg + 1, (value >> 8) & 0xFF);
}

int ms7210_dvin_timing_config(ms7210_dev_t *dev, struct dvin_config *dc, struct videotiming *vt, enum HDMI_CLK_REPEAT *rpt)
{
    uint8_t val = 0;
    unsigned int pix_shift = 3, line_shift = 2;
    int clkx2 = 0;
    struct videotiming svt;

    // Read clock select register
    ms7210_read(dev, MS7210_MISC_DIG_CLK_SEL_REG, &val);
    if (val & 1) {
        clkx2 = 1;
        vt->htotal /= *rpt;
        vt->hsyncwidth /= *rpt;
        vt->hoffset /= *rpt;
        vt->hactive /= *rpt;
        vt->pixclk /= *rpt;
        *rpt = 0;
    }

    // Copy timing parameters
    svt = *vt;

    if (*rpt) {
        clkx2 = 1;
        svt.htotal /= *rpt;
        svt.hsyncwidth /= *rpt;
        svt.hoffset /= *rpt;
        svt.hactive /= *rpt;
        svt.pixclk /= *rpt;
        *rpt = 1;
    }

    // Adjust timing for 8/10/12-bit SDR mode
    if ((dc->bw_mode == DVIN_BW_MODE_8_10_12BIT) &&
        (dc->dr_mode == DVIN_DR_MODE_SDR)) {
        svt.htotal *= 2;
        svt.hsyncwidth *= 2;
        svt.hoffset *= 2;
        svt.hactive *= 2;
    }

    // Calculate pixel and line shifts based on sync mode
    switch (dc->sy_mode) {
        case DVIN_SY_MODE_HSVS:
            break;
        case DVIN_SY_MODE_HSVSDE:
        case DVIN_SY_MODE_VSDE:
            pix_shift += svt.hoffset;
            break;
        case DVIN_SY_MODE_DEONLY:
            pix_shift += svt.hoffset;
            line_shift += svt.voffset;
            break;
        case DVIN_SY_MODE_EMBEDDED:
            pix_shift += svt.hoffset + svt.hactive;
            line_shift += svt.voffset + svt.vactive / ((svt.polarity & 0x01) ? 1 : 2);
            break;
        case DVIN_SY_MODE_2XEMBEDDED:
            pix_shift += svt.hoffset + svt.hactive + 4;
            line_shift += svt.voffset + svt.vactive / ((svt.polarity & 0x01) ? 1 : 2);
            break;
        case DVIN_SY_MODE_BTAT1004:
            pix_shift += svt.hoffset + svt.hactive - 2;
            line_shift += svt.voffset + svt.vactive / ((svt.polarity & 0x01) ? 1 : 2);
            break;
    }

    // Write timing parameters to registers
    ms7210_write16(dev, MS7210_DVIN_HT_L_REG, svt.htotal);
    ms7210_write16(dev, MS7210_DVIN_VT_L_REG, svt.vtotal);
    ms7210_write16(dev, MS7210_DVIN_PXL_SHIFT_L_REG, pix_shift);
    ms7210_write16(dev, MS7210_DVIN_LN_SHIFT_L_REG, line_shift);
    ms7210_write16(dev, MS7210_DVIN_HSW_L_REG, svt.hsyncwidth);
    ms7210_write16(dev, MS7210_DVIN_VSW_L_REG, svt.vsyncwidth);
    ms7210_write16(dev, MS7210_DVIN_HDE_ST_L_REG, svt.hoffset);
    ms7210_write16(dev, MS7210_DVIN_HDE_SP_L_REG, svt.hoffset + svt.hactive);
    ms7210_write16(dev, MS7210_DVIN_VDE_OST_L_REG, svt.voffset);
    ms7210_write16(dev, MS7210_DVIN_VDE_OSP_L_REG,
                   svt.voffset + svt.vactive / ((svt.polarity & 0x01) ? 1 : 2));
    ms7210_write16(dev, MS7210_DVIN_VDE_EST_L_REG,
                   svt.voffset + svt.vtotal / 2 + 1);
    ms7210_write16(dev, MS7210_DVIN_VDE_ESP_L_REG,
                   svt.voffset + svt.vtotal / 2 + svt.vactive / 2 + 1);

    // Configure sync polarities
    ms7210_update_bits(dev, MS7210_DVIN_SYNCIN_FLIP_REG,
                       0x01, (svt.polarity & 0x02) ? 0x00 : 0x01);
    ms7210_update_bits(dev, MS7210_DVIN_SYNCIN_FLIP_REG,
                       0x02, (svt.polarity & 0x04) ? 0x00 : 0x02);
    ms7210_update_bits(dev, MS7210_DVIN_SYNCOUT_FLIP_REG,
                       0x10, (svt.polarity & 0x02) ? 0x00 : 0x10);
    ms7210_update_bits(dev, MS7210_DVIN_SYNCOUT_FLIP_REG,
                       0x20, (svt.polarity & 0x04) ? 0x00 : 0x20);
    ms7210_update_bits(dev, MS7210_DVIN_SYNCOUT_FLIP_REG,
                       0x04, (svt.polarity & 0x01) ? 0x04 : 0x00);

    return clkx2;
}

void ms7210_hdmi_tx_output_config(ms7210_dev_t *dev, struct hdmi_config *hc)
{
    // Disable output and HDCP
    ms7210_hdmi_tx_phy_output_enable(dev, false);
    ms7210_hdmi_tx_hdcp_enable(dev, false);
    ms7210_hdmi_tx_shell_set_gcp_packet_avmute(dev, false);

    // Configure color space
    ms7210_csc_config_output(dev, hc->color_space);

    // Configure PHY and transmitter
    ms7210_hdmi_tx_phy_config(dev, hc->video_clk);
    ms7210_hdmi_tx_shell_config(dev, hc);

    // Enable output
    ms7210_hdmi_tx_shell_video_mute_enable(dev, false);
    ms7210_hdmi_tx_shell_audio_mute_enable(dev, false);
    ms7210_hdmi_tx_phy_output_enable(dev, true);
}