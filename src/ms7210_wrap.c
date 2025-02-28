#include "ms7210_wrap.h"

static int ms7210_check(ms7210_dev_t *dev, uint16_t reg, uint8_t write_value)
{
    uint8_t read_value;
    int ret;
    
    ret = ms7210_read(dev, reg, &read_value);
    if (ret < 0) {
        printf("Check reg 0x%04x failed - read error\n", reg);
        return ret;
    }
    
    if (read_value != write_value) {
        printf("Check reg 0x%04x failed - write: 0x%02x, read: 0x%02x\n",
               reg, write_value, read_value);
        return -1;
    }
    
    #if 1
    printf("Check reg 0x%04x success - value: 0x%02x\n", reg, read_value);
    #endif
    
    return 0;
}

#if 1
int ms7210_write(ms7210_dev_t *dev, uint16_t reg, uint8_t value)
{
    uint8_t buf[3];
    struct i2c_msg messages[1];
    struct i2c_rdwr_ioctl_data packets;
    int ret;

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

    // ret = ms7210_check(dev, reg, value);
    // if(ret < 0) {
    //     return ret;
    // }

    return 0;
}
#else
int ms7210_write(ms7210_dev_t *dev, uint16_t reg, uint8_t value)
{
    uint8_t buf[3];
    struct i2c_msg messages[1];
    struct i2c_rdwr_ioctl_data packets;
    int ret;
    int retry_count = 0;
    const int MAX_RETRIES = 3;  // 最大重试次数
    const int RETRY_DELAY_US = 1000;  // 重试延时1ms

    do {
        // 如果是重试，打印重试信息
        if (retry_count > 0) {
            printf("Retrying write reg 0x%04x (attempt %d/%d)...\n", 
                   reg, retry_count + 1, MAX_RETRIES);
        }

        // 准备I2C消息
        buf[0] = reg & 0xFF; 
        buf[1] = (reg >> 8) & 0xFF;
        buf[2] = value;

        messages[0].addr = MS7210_I2C_ADDR;
        messages[0].flags = 0;
        messages[0].len = 3;
        messages[0].buf = buf;

        packets.msgs = messages;
        packets.nmsgs = 1;

        // 执行I2C写入
        if(ioctl(dev->i2c_fd, I2C_RDWR, &packets) < 0) {
            printf("Write reg 0x%04x failed (attempt %d/%d): %s\n", 
                   reg, retry_count + 1, MAX_RETRIES, strerror(errno));
            
            // 如果已达到最大重试次数，返回错误
            if (retry_count >= MAX_RETRIES - 1) {
                printf("Max retries reached for reg 0x%04x, giving up\n", reg);
                return -1;
            }
            
            // 否则等待一段时间后重试
            usleep(RETRY_DELAY_US);
            retry_count++;
            continue;
        }

        // 验证写入
        ret = ms7210_check(dev, reg, value);
        if(ret == 0) {
            // 如果之前有重试，打印成功信息
            if (retry_count > 0) {
                printf("Write reg 0x%04x succeeded after %d retries\n", 
                       reg, retry_count);
            }
            return 0;  // 写入成功，返回
        }

        // 验证失败，如果未达到最大重试次数则重试
        if (retry_count >= MAX_RETRIES - 1) {
            printf("Max retries reached for reg 0x%04x, giving up\n", reg);
            return ret;
        }

        printf("Verification failed for reg 0x%04x (attempt %d/%d)\n", 
               reg, retry_count + 1, MAX_RETRIES);
        usleep(RETRY_DELAY_US);
        retry_count++;

    } while (retry_count < MAX_RETRIES);

    // 这里正常不会执行到，为了代码完整性添加
    return -1;
}
#endif

int ms7210_write16(ms7210_dev_t *dev, uint16_t reg, uint16_t value)
{
    int err;
    
    if ((err = ms7210_write(dev, reg, value & 0xFF)) < 0)
        return err;
        
    return ms7210_write(dev, reg + 1, (value >> 8) & 0xFF);
}

int ms7210_read(ms7210_dev_t *dev, uint16_t reg, uint8_t *value)
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

int ms7210_update_bits(ms7210_dev_t *dev, uint16_t reg, uint8_t mask, uint8_t value)
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

void ms7210_rc_freq_set(ms7210_dev_t *dev)
{
    ms7210_write(dev, MS7210_RC_CTRL1_REG, 0x81);
    ms7210_write(dev, MS7210_RC_CTRL1_REG, 0x54);
}

void ms7210_csc_config_input(ms7210_dev_t *dev)
{
    ms7210_update_bits(dev, MS7210_CSC_CTRL1_REG, 0x03, 0x10 | dev->cs_mode);
}

void ms7210_dig_pads_pull_set(ms7210_dev_t *dev, unsigned int pull)
{
    ms7210_update_bits(dev, MS7210_IO_CTRL3_REG, 0x30, (pull & 3) << 4);
}

void ms7210_misc_audio_pad_in_spdif(ms7210_dev_t *dev)
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

// CSC配置
void ms7210_csc_config_output(ms7210_dev_t *dev, enum HDMI_CS cs)
{
    unsigned int c;

    switch (cs) {
        case HDMI_RGB:
            c = 0;
            break;
        case HDMI_YCBCR444:
            c = 1;
            break;
        case HDMI_YCBCR422:
            c = 2;
            break;
        default:
            c = 3;
            break;
    }
    ms7210_update_bits(dev, MS7210_CSC_CTRL1_REG, 0x0c, c << 2);
}

// PHY输出使能
void ms7210_hdmi_tx_phy_output_enable(ms7210_dev_t *dev, bool enable)
{
    if (enable) {
        /* output data drive control */
        ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_DATA_DRV_REG + 
                          dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                          0x07, 0x07);
        /* output clk drive control */
        ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_POWER_REG +
                          dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                          0x04, 0x04);
    } else {
        /* output clk drive control */
        ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_POWER_REG +
                          dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                          0x04, 0);
        /* output data drive control */
        ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_DATA_DRV_REG +
                          dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                          0x07, 0);
    }
}

// HDCP使能
void ms7210_hdmi_tx_hdcp_enable(ms7210_dev_t *dev, bool enable)
{
    if (enable) {
        ms7210_write(dev, MS7210_HDMI_TX_HDCP_CONTROL_REG +
                     dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST, 0x0b);
    } else {
        ms7210_write(dev, MS7210_HDMI_TX_HDCP_CONTROL_REG +
                     dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST, 0x04);
        ms7210_write(dev, MS7210_HDMI_TX_HDCP_CONTROL_REG +
                     dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST, 0x00);
    }
}

// GCP包静音设置
void ms7210_hdmi_tx_shell_set_gcp_packet_avmute(ms7210_dev_t *dev, bool mute)
{
    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_CTRL_REG +
                       dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                       0x80, (mute) ? 0x80 : 0x00);
}

// 视频静音使能
void ms7210_hdmi_tx_shell_video_mute_enable(ms7210_dev_t *dev, bool en)
{
    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_AVMUTE_REG +
                       dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                       0x02, en ? 0x02 : 0x00);
}

// 音频静音使能
void ms7210_hdmi_tx_shell_audio_mute_enable(ms7210_dev_t *dev, bool en)
{
    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_AVMUTE_REG +
                       dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                       0x04, en ? 0x04 : 0x00);
}

int ms7210_dvin_timing_config(ms7210_dev_t *dev, struct dvin_config *dc,
                             struct videotiming *vt, enum HDMI_CLK_REPEAT *rpt)
{
    printf("\n=== Starting DVIN Timing Configuration ===\n");
    printf("Initial timing parameters:\n");
    printf("- H Total: %d\n", vt->htotal);
    printf("- V Total: %d\n", vt->vtotal);
    printf("- H Active: %d\n", vt->hactive);
    printf("- V Active: %d\n", vt->vactive);
    printf("- Pixel Clock: %d\n", vt->pixclk);
    printf("- V Frequency: %d\n", vt->vfreq);
    
    uint8_t val = 0;
    unsigned int pix_shift = 3, line_shift = 2;
    int clkx2 = 0;
    struct videotiming svt;

    ms7210_read(dev, MS7210_MISC_DIG_CLK_SEL_REG, &val);
    printf("Clock select register value: 0x%02x\n", val);
    
    if (val & 1) {
        printf("Clock x2 mode detected, adjusting parameters...\n");
        clkx2 = 1;
        vt->htotal /= *rpt;
        vt->hsyncwidth /= *rpt;
        vt->hoffset /= *rpt;
        vt->hactive /= *rpt;
        vt->pixclk /= *rpt;
        *rpt = 0;
    }

    svt = *vt;

    if (*rpt) {
        printf("Applying clock repeat factor...\n");
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

    printf("=== DVIN Timing Configuration Complete ===\n");
    printf("Final clock x2 status: %d\n\n", clkx2);

    return clkx2;
}

void ms7210_hdmi_tx_phy_config(ms7210_dev_t *dev, unsigned int video_clk)
{
    printf("\n=== Starting PHY Configuration ===\n");
    
    printf("Selecting clock source...\n");
    ms7210_hdmi_tx_clk_sel(dev, 1);

    printf("Initializing PHY with TMDS clock: %d\n", video_clk);
    ms7210_hdmi_tx_phy_init(dev, video_clk);
    
    printf("Enabling PHY power...\n");
    ms7210_hdmi_tx_phy_power_enable(dev, true);
    
    printf("Waiting for PLL power stability...\n");
    usleep(10000);
    
    printf("Setting PHY clock...\n");
    ms7210_hdmi_tx_phy_set_clk(dev, video_clk);
    
    printf("=== PHY Configuration Complete ===\n\n");
}

void ms7210_hdmi_tx_output_config(ms7210_dev_t *dev, struct hdmi_config *hc)
{
    printf("\n=== Starting HDMI TX Output Configuration ===\n");
    
    printf("Disabling PHY output and HDCP...\n");
    ms7210_hdmi_tx_phy_output_enable(dev, false);
    ms7210_hdmi_tx_hdcp_enable(dev, false);
    ms7210_hdmi_tx_shell_set_gcp_packet_avmute(dev, false);

    printf("Configuring CSC for color space: %d\n", hc->color_space);
    ms7210_csc_config_output(dev, hc->color_space);

    printf("Configuring PHY with video clock: %d\n", hc->video_clk);
    ms7210_hdmi_tx_phy_config(dev, hc->video_clk);
    
    printf("Configuring HDMI Shell...\n");
    ms7210_hdmi_tx_shell_config(dev, hc);

    printf("Enabling output...\n");
    ms7210_hdmi_tx_shell_video_mute_enable(dev, false);
    ms7210_hdmi_tx_shell_audio_mute_enable(dev, false);
    ms7210_hdmi_tx_phy_output_enable(dev, true);
    
    printf("=== HDMI TX Output Configuration Complete ===\n\n");
}
// HDMI Shell配置主函数
void ms7210_hdmi_tx_shell_config(ms7210_dev_t *dev, struct hdmi_config *hc)
{
    printf("\n=== Starting Shell Configuration ===\n");
    
    printf("Enabling shell reset...\n");
    ms7210_hdmi_tx_shell_reset_enable(dev, true);
    
    printf("Initializing shell...\n");
    ms7210_hdmi_tx_shell_init(dev);
    
    printf("Setting HDMI/DVI mode: %s\n", hc->hdmi_flag ? "HDMI" : "DVI");
    ms7210_hdmi_tx_shell_set_hdmi_out(dev, hc->hdmi_flag);
    
    printf("Setting clock repeat: %d\n", hc->clk_rpt);
    ms7210_hdmi_tx_shell_set_clk_repeat(dev, hc->clk_rpt);

    printf("Setting color space: %d\n", hc->color_space);
    if (hc->color_space == HDMI_YCBCR422 && hc->color_depth != HDMI_COLOR_DEPTH_8BIT) {
        printf("YUV422 with deep color detected, forcing RGB mode\n");
        ms7210_hdmi_tx_shell_set_color_space(dev, HDMI_RGB);
    } else {
        ms7210_hdmi_tx_shell_set_color_space(dev, hc->color_space);
    }

    printf("Setting color depth: %d\n", hc->color_depth);
    ms7210_hdmi_tx_shell_set_color_depth(dev, hc->color_depth);

    printf("Configuring audio parameters...\n");
    printf("- Rate: %d\n", hc->audio_rate);
    printf("- Bits: %d\n", hc->audio_bits);
    printf("- Channels: %d\n", hc->audio_channels);
    ms7210_hdmi_tx_shell_set_audio_rate(dev, hc->audio_rate);
    ms7210_hdmi_tx_shell_set_audio_bits(dev, hc->audio_bits);
    ms7210_hdmi_tx_shell_set_audio_channels(dev, hc->audio_channels);

    printf("Disabling shell reset...\n");
    ms7210_hdmi_tx_shell_reset_enable(dev, false);

    printf("Setting InfoFrames...\n");
    ms7210_hdmi_tx_shell_set_video_infoframe(dev, hc);
    ms7210_hdmi_tx_shell_set_audio_infoframe(dev, hc);
    if (hc->video_format) {
        printf("Setting vendor specific InfoFrame...\n");
        ms7210_hdmi_tx_shell_set_vendor_specific_infoframe(dev, hc);
    }
    
    printf("=== Shell Configuration Complete ===\n\n");
}

// Shell初始化
void ms7210_hdmi_tx_shell_init(ms7210_dev_t *dev)
{
    uint16_t reg_base = dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST;

    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_VOLUME_CFG0_REG + reg_base, 
                       0x01, 0x01);

    /* video and audio clk enable */
    ms7210_update_bits(dev, MS7210_HDMI_TX_MISC_HDMI_CLK_REG,
                       0x11 << dev->tx_channel,
                       0x11 << dev->tx_channel);

    /* audio */
    ms7210_update_bits(dev, MS7210_HDMI_TX_MISC_AUD_CTRL, 0x0f, 0x00);
    ms7210_update_bits(dev, MS7210_HDMI_TX_AUDIO_RATE_REG + reg_base, 
                       0x80, 0x80);
}

// 设置HDMI/DVI输出模式
void ms7210_hdmi_tx_shell_set_hdmi_out(ms7210_dev_t *dev, bool hdmi_out)
{
    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_DVI_REG + 
                       dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                       0x01, (hdmi_out) ? 0x00 : 0x01);
}

// 设置时钟重复
void ms7210_hdmi_tx_shell_set_clk_repeat(ms7210_dev_t *dev, unsigned int rpt)
{
    if (rpt > HDMI_X10CLK)
        rpt = HDMI_X1CLK;

    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_MODE_REG + 
                       dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                       0x30, rpt << 4);
}

// 设置色彩空间
void ms7210_hdmi_tx_shell_set_color_space(ms7210_dev_t *dev, unsigned int cs)
{
    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_MODE_REG + 
                       dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                       0xc0, cs << 6);
}

// 设置色深
void ms7210_hdmi_tx_shell_set_color_depth(ms7210_dev_t *dev, unsigned int depth)
{
    uint16_t reg = MS7210_HDMI_TX_SHELL_MODE_REG + 
                   dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST;

    ms7210_update_bits(dev, reg, 0x04, (depth > 0) ? 0x04 : 0x00);
    ms7210_update_bits(dev, reg, 0x03, depth);
}

// 设置音频采样率
void ms7210_hdmi_tx_shell_set_audio_rate(ms7210_dev_t *dev, unsigned int audio_rate)
{
    uint16_t reg = MS7210_HDMI_TX_AUDIO_RATE_REG + 
                   dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST;
    uint8_t rate_val;

    switch (audio_rate) {
        case HDMI_AUD_RATE_96K:
            rate_val = 0x02;
            break;
        case HDMI_AUD_RATE_32K:
            rate_val = 0x40;
            break;
        case HDMI_AUD_RATE_88K2:
            rate_val = 0x10;
            break;
        case HDMI_AUD_RATE_176K4:
            rate_val = 0x08;
            break;
        case HDMI_AUD_RATE_192K:
            rate_val = 0x01;
            break;
        case HDMI_AUD_RATE_44K1:
        case HDMI_AUD_RATE_48K:
        default:
            rate_val = 0x04;
            break;
    }

    ms7210_update_bits(dev, reg, 0x7F, rate_val);
}

// 设置音频位深
void ms7210_hdmi_tx_shell_set_audio_bits(ms7210_dev_t *dev, unsigned int audio_bits)
{
    ms7210_update_bits(dev, MS7210_HDMI_TX_AUDIO_CFG_I2S_REG + 
                       dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                       0x30, audio_bits ? 0x00 : 0x10);
}

// 设置音频通道
void ms7210_hdmi_tx_shell_set_audio_channels(ms7210_dev_t *dev, unsigned int audio_channels)
{
    uint8_t ch_val;

    if (audio_channels <= HDMI_AUD_2CH)
        ch_val = 0x01;
    else if (audio_channels <= HDMI_AUD_4CH)
        ch_val = 0x03;
    else if (audio_channels <= HDMI_AUD_6CH)
        ch_val = 0x07;
    else
        ch_val = 0x0f;

    ms7210_update_bits(dev, MS7210_HDMI_TX_AUDIO_CH_EN_REG + 
                       dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                       0x0f, ch_val);
}

// 设置视频信息帧
void ms7210_hdmi_tx_shell_set_video_infoframe(ms7210_dev_t *dev, struct hdmi_config *hc)
{
    int i;
    uint8_t frame[14];
    uint16_t reg_base = dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST;

    /* "AVI packet is enabled, active is high" */
    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_CTRL_REG + reg_base, 
                       0x40, 0x00);

    memset(frame, 0, sizeof(frame));

    // 配置AVI信息帧
    frame[1] = ((hc->color_space << 5) & 0x60) |
               0x10 | /* Active Format Information Present, Active */
               (hc->scan_info & 0x03);

    // 配置色度
    if (hc->colorimetry == HDMI_COLORIMETRY_601) {
        frame[2] = 0x40;
    } else if ((hc->colorimetry == HDMI_COLORIMETRY_709) || 
               (hc->colorimetry == HDMI_COLORIMETRY_1120)) {
        frame[2] = 0x80;
    } else if (hc->colorimetry == HDMI_COLORIMETRY_XVYCC601) {
        frame[2] = 0xc0;
        frame[3] &= ~0x70;
    } else if (hc->colorimetry == HDMI_COLORIMETRY_XVYCC709) {
        frame[2] = 0xc0;
        frame[3] = 0x10;
    } else {
        frame[2] &= ~0xc0;
        frame[3] &= ~0x70;
    }

    // 配置宽高比
    if (hc->aspect_ratio == HDMI_4X3)
        frame[2] |= 0x10;
    else if (hc->aspect_ratio == HDMI_16X9)
        frame[2] |= 0x20;

    frame[2] |= 0x08;  // 默认宽高比编码

    // 配置VIC
    frame[4] = (hc->vic >= 64) ? 0 : hc->vic;
    frame[5] = hc->clk_rpt;

    // 计算校验和
    frame[0] = 0x82 + 0x02 + 0x0D;
    for (i = 1; i < 14; i++)
        frame[0] += frame[i];
    frame[0] = 0x100 - frame[0];

    // 写入信息帧
    ms7210_write(dev, MS7210_HDMI_TX_SHELL_INFO_TYPE_REG + reg_base, 0x82);
    ms7210_write(dev, MS7210_HDMI_TX_SHELL_INFO_VER_REG + reg_base, 0x02);
    ms7210_write(dev, MS7210_HDMI_TX_SHELL_INFO_LEN_REG + reg_base, 0x0D);

    for (i = 0; i < 14; i++)
        ms7210_write(dev, MS7210_HDMI_TX_SHELL_INFO_PACK_REG + reg_base, 
                     frame[i]);

    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_CTRL_REG + reg_base, 
                       0x40, 0x40);
}

// 设置音频信息帧
void ms7210_hdmi_tx_shell_set_audio_infoframe(ms7210_dev_t *dev, struct hdmi_config *hc)
{
    int i;
    uint8_t frame[14];
    uint16_t reg_base = dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST;

    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_CTRL_REG + reg_base, 
                       0x20, 0x00);

    memset(frame, 0, sizeof(frame));
    frame[0] = 0x84;
    frame[1] = 0x01;
    frame[2] = 0x0A;
    frame[3] = 0x84 + 0x01 + 0x0A;
    frame[4] = hc->audio_channels;
    frame[5] = 0;
    frame[7] = hc->audio_speaker_locations;

    // 计算校验和
    for (i = 4; i < 14; i++)
        frame[3] += frame[i];
    frame[3] = 0x100 - frame[3];

    // 写入信息帧
    ms7210_write(dev, MS7210_HDMI_TX_SHELL_INFO_TYPE_REG + reg_base, frame[0]);
    ms7210_write(dev, MS7210_HDMI_TX_SHELL_INFO_VER_REG + reg_base, frame[1]);
    ms7210_write(dev, MS7210_HDMI_TX_SHELL_INFO_LEN_REG + reg_base, frame[2]);

    for (i = 3; i < 14; i++)
        ms7210_write(dev, MS7210_HDMI_TX_SHELL_INFO_PACK_REG + reg_base, 
                     frame[i]);

    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_CTRL_REG + reg_base, 
                       0x20, 0x20);
}

// 设置厂商特定信息帧
void ms7210_hdmi_tx_shell_set_vendor_specific_infoframe(ms7210_dev_t *dev, struct hdmi_config *hc)
{
    int i;
    uint8_t frame[32];
    uint16_t reg_base = dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST;

    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_CFG_REG + reg_base, 
                       0x01, 0x00);

    memset(frame, 0, sizeof(frame));
    frame[0] = 0x81;
    frame[1] = 0x01;
    frame[2] = 0x1B;
    frame[3] = 0x81 + 0x01 + 0x1B;
    frame[4] = 0x03;  // 24bit IEEE
    frame[5] = 0x0C;
    frame[6] = 0x00;
    frame[7] = hc->video_format << 5;  // HDMI_VIDEO_FORMAT
    frame[8] = hc->h4Kx2K_vic;  // HDMI_VIC

    // 计算校验和
    for (i = 4; i < 31; i++)
        frame[3] += frame[i];
    frame[3] = 0x100 - frame[3];

    // 写入信息帧
    ms7210_write(dev, MS7210_HDMI_TX_SHELL_INFO_TYPE_REG + reg_base, frame[0]);
    ms7210_write(dev, MS7210_HDMI_TX_SHELL_INFO_VER_REG + reg_base, frame[1]);
    ms7210_write(dev, MS7210_HDMI_TX_SHELL_INFO_LEN_REG + reg_base, frame[2]);

    for (i = 3; i < 31; i++)
        ms7210_write(dev, MS7210_HDMI_TX_SHELL_INFO_PACK_REG + reg_base, 
                     frame[i]);

    ms7210_update_bits(dev, MS7210_HDMI_TX_SHELL_CFG_REG + reg_base, 
                       0x01, 0x01);
}

// 时钟源选择
void ms7210_hdmi_tx_clk_sel(ms7210_dev_t *dev, unsigned int sel)
{
    switch (sel) {
        case 0:
            /* clk from rx */
            ms7210_update_bits(dev, MS7210_MISC_CLK_CTRL4_REG,
                              0x0c, 1 << 2);
            break;
        case 1:
            /* clk from dvin */
            ms7210_update_bits(dev, MS7210_MISC_CLK_CTRL4_REG,
                              0x0d, (2 << 2) | 0x01);
            ms7210_update_bits(dev, MS7210_HDMI_TX_PLL_CTRL1_REG + 
                              dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                              0x18, 0 << 3);
            break;
    }
}

// PHY初始化
void ms7210_hdmi_tx_phy_init(ms7210_dev_t *dev, unsigned int tmds)
{
    unsigned int main_po, post_po;
    uint16_t reg_base = dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST;

    /* tmds_clk > 200MHz */
    main_po = (tmds > 20000) ? 9 : 4;
    post_po = (tmds > 20000) ? 9 : 7;

    /* PLL init, use Hardware auto mode */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PLL_CFG_SEL_REG + reg_base, 
                       0x07, 0x00);
    ms7210_update_bits(dev, MS7210_HDMI_TX_PLL_CTRL1_REG + reg_base, 
                       0x01, 0x01);
    ms7210_update_bits(dev, MS7210_HDMI_TX_PLL_CTRL4_REG + reg_base, 
                       0x08, 0x08);

    /* misc clk and reset */
    ms7210_update_bits(dev, MS7210_HDMI_TX_MISC_ACLK_SEL_REG, 0xf0, 0xf0);
    ms7210_update_bits(dev, MS7210_HDMI_TX_MISC_RST_CTRL1_REG, 0xf0, 0xf0);

    /* clk drive */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_MAIN_PREC_2_REG + reg_base,
                       0x70, 4 << 4); /* clk main pre */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_MAIN_POC_2_REG + reg_base,
                       0xf0, 2 << 4); /* clk main po */

    /* data0 drive */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_MAIN_PRE0_1_REG + reg_base,
                       0x07, 4 << 0); /* data main pre */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_MAIN_PO0_1_REG + reg_base,
                       0x0f, main_po << 0); /* data main po */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_POST_PRE_REG + reg_base,
                       0x01, 1 << 0); /* data post pre */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_POST_PO0_1_REG + reg_base,
                       0x0f, post_po << 0); /* data post po */

    /* data1 drive */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_MAIN_PRE0_1_REG + reg_base,
                       0x70, 4 << 4); /* data main pre */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_MAIN_PO0_1_REG + reg_base,
                       0xf0, main_po << 4); /* data main po */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_POST_PRE_REG + reg_base,
                       0x02, 1 << 1); /* data post pre */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_POST_PO0_1_REG + reg_base,
                       0xf0, post_po << 4); /* data post po */

    /* data2 drive */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_MAIN_PREC_2_REG + reg_base,
                       0x07, 4 << 0); /* data main pre */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_MAIN_POC_2_REG + reg_base,
                       0x07, main_po << 0); /* data main po */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_POST_PRE_REG + reg_base,
                       0x04, 1 << 2); /* data post pre */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_POST_PO2_REG + reg_base,
                       0x0f, post_po << 0); /* data post po */

    ms7210_write(dev, MS7210_HDMI_TX_PLL_POWER_REG + reg_base, 0x20);

    /* High speed configuration for TMDS > 10GHz */
    if ((tmds / 100) > 100) {
        ms7210_update_bits(dev, MS7210_HDMI_TX_PLL_CTRL6_REG + reg_base,
                          0x04, 0x04);
        ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_POWER_REG + reg_base,
                          0x40, 0x40);
        ms7210_write(dev, MS7210_HDMI_TX_PHY_MAIN_PO0_1_REG + reg_base, 0xdd);
        ms7210_write(dev, MS7210_HDMI_TX_PHY_MAIN_POC_2_REG + reg_base, 0x0d);
        ms7210_write(dev, MS7210_HDMI_TX_PHY_POST_PO0_1_REG + reg_base, 0x88);
        ms7210_write(dev, MS7210_HDMI_TX_PHY_POST_PO2_REG + reg_base, 0x08);
    } else {
        ms7210_update_bits(dev, MS7210_HDMI_TX_PLL_CTRL6_REG + reg_base, 
                          0x04, 0x00);
        ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_POWER_REG + reg_base, 
                          0x40, 0x00);
        ms7210_write(dev, MS7210_HDMI_TX_PHY_MAIN_PO0_1_REG + reg_base, 0x11);
        ms7210_write(dev, MS7210_HDMI_TX_PHY_MAIN_POC_2_REG + reg_base, 0x01);
        ms7210_write(dev, MS7210_HDMI_TX_PHY_POST_PO0_1_REG + reg_base, 0x11);
        ms7210_write(dev, MS7210_HDMI_TX_PHY_POST_PO2_REG + reg_base, 0x01);
    }

    /* HBR audio mode clock configuration (<= 350MHz) */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PLL_CTRL5_REG + reg_base,
                       0x03, 0x02);
}

// PHY电源控制
void ms7210_hdmi_tx_phy_power_enable(ms7210_dev_t *dev, bool enable)
{
    uint16_t reg_base = dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST;

    /* PLL power control */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PLL_POWER_REG + reg_base, 
                       0x07, (enable) ? 0 : 0x07);

    /* PHY power control */
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_POWER_REG + reg_base, 
                       0x0a, (enable) ? 0x0a : 0);
    ms7210_update_bits(dev, MS7210_HDMI_TX_PHY_POWER_REG + reg_base, 
                       0x20, (enable) ? 0 : 0x20);
}

// 设置时钟比率
void ms7210_hdmi_tx_phy_set_clk_ratio(ms7210_dev_t *dev, int ratio)
{
    unsigned int val = 0;

    switch (ratio) {
        case 0:  // Normal clock
            val = 0;
            break;
        case 1:  // Double clock (x2)
            val = 0x04;
            break;
        case 2:  // Half clock (1/2)
            val = 0x40;
            break;
    }
    ms7210_update_bits(dev, MS7210_HDMI_TX_PLL_CTRL11_REG +
                       dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
                       0x44, val);
}

// Shell复位控制
void ms7210_hdmi_tx_shell_reset_enable(ms7210_dev_t *dev, bool en)
{
    /* video and audio reset */
    ms7210_update_bits(dev, MS7210_HDMI_TX_MISC_HDMI_RST_REG,
                       0x11 << dev->tx_channel,
                       (en) ? 0x00 : (0x11 << dev->tx_channel));
}

void ms7210_hdmi_tx_phy_set_clk(ms7210_dev_t *dev, unsigned int clk)
{
    // 触发PLL配置
    ms7210_write(dev, MS7210_HDMI_TX_PLL_TRIG_REG + 
                 dev->tx_channel * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST, 
                 0x01);
    
    // 等待PLL时钟稳定
    // 要求延时>100us, 这里使用10ms以确保足够的稳定时间
    usleep(10000);  // 10ms = 10000us
}