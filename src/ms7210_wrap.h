
#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include "ms7210.h"

#ifdef __cplusplus
extern "C" {
#endif


enum ms7210_type {
    MS7210 = 0,
};

enum DVIN_CS_MODE {
    DVIN_CS_MODE_RGB,
    DVIN_CS_MODE_YUV444,
    DVIN_CS_MODE_YUV422
};

enum DVIN_BW_MODE {
    DVIN_BW_MODE_16_20_24BIT,
    DVIN_BW_MODE_8_10_12BIT
};

enum DVIN_SQ_MODE {
    DVIN_SQ_MODE_NONSEQ,
    DVIN_SQ_MODE_SEQ
};

enum DVIN_DR_MODE {
    DVIN_DR_MODE_SDR,
    DVIN_DR_MODE_DDR
};

enum DVIN_SY_MODE {
    DVIN_SY_MODE_HSVSDE,      // 8/16/24-bit BT601
    DVIN_SY_MODE_HSVS,
    DVIN_SY_MODE_VSDE,        // non suport interlace mode
    DVIN_SY_MODE_DEONLY,
    DVIN_SY_MODE_EMBEDDED,    // 16-bit BT1120 or 8bit BT656
    DVIN_SY_MODE_2XEMBEDDED,  // 8-bit BT1120
    DVIN_SY_MODE_BTAT1004     // 16-bit BTA-T1004
};

/* HDMI audio */
enum HDMI_AUDIO_MODE {
    HDMI_AUD_MODE_AUDIO_SAMPLE  = 0x00,
    HDMI_AUD_MODE_HBR           = 0x01,
    HDMI_AUD_MODE_DSD           = 0x02,
    HDMI_AUD_MODE_DST           = 0x03
};

enum HDMI_AUDIO_RATE {
    HDMI_AUD_RATE_44K1  = 0x00,
    HDMI_AUD_RATE_48K   = 0x02,
    HDMI_AUD_RATE_32K   = 0x03,
    HDMI_AUD_RATE_88K2  = 0x08,
    HDMI_AUD_RATE_96K   = 0x0A,
    HDMI_AUD_RATE_176K4 = 0x0C,
    HDMI_AUD_RATE_192K  = 0x0E
};

enum HDMI_AUDIO_LENGTH {
    HDMI_AUD_LENGTH_16BITS    = 0x00,
    HDMI_AUD_LENGTH_20BITS    = 0x01,
    HDMI_AUD_LENGTH_24BITS    = 0x02
};

enum HDMI_AUDIO_CHN {
    HDMI_AUD_2CH    = 0x01,
    HDMI_AUD_3CH    = 0x02,
    HDMI_AUD_4CH    = 0x03,
    HDMI_AUD_5CH    = 0x04,
    HDMI_AUD_6CH    = 0x05,
    HDMI_AUD_7CH    = 0x06,
    HDMI_AUD_8CH    = 0x07
};

enum HDMI_CLK_REPEAT {
    HDMI_X1CLK      = 0x00,
    HDMI_X2CLK      = 0x01,
    HDMI_X3CLK      = 0x02,
    HDMI_X4CLK      = 0x03,
    HDMI_X5CLK      = 0x04,
    HDMI_X6CLK      = 0x05,
    HDMI_X7CLK      = 0x06,
    HDMI_X8CLK      = 0x07,
    HDMI_X9CLK      = 0x08,
    HDMI_X10CLK     = 0x09
};

enum HDMI_SCAN_INFO {
    HDMI_OVERSCAN     = 0x01,    //television type
    HDMI_UNDERSCAN    = 0x02     //computer type
};

enum HDMI_ASPECT_RATIO {
    HDMI_4X3     = 0x01,
    HDMI_16X9    = 0x02
};

enum HDMI_CS {
    HDMI_RGB        = 0x00,
    HDMI_YCBCR422   = 0x01,
    HDMI_YCBCR444   = 0x02,
    HDMI_YUV420     = 0x03
};

enum HDMI_COLOR_DEPTH {
    HDMI_COLOR_DEPTH_8BIT    = 0x00,
    HDMI_COLOR_DEPTH_10BIT   = 0x01,
    HDMI_COLOR_DEPTH_12BIT   = 0x02,
    HDMI_COLOR_DEPTH_16BIT   = 0x03
};

enum HDMI_COLORIMETRY {
    HDMI_COLORIMETRY_601    = 0x00,
    HDMI_COLORIMETRY_709    = 0x01,
    HDMI_COLORIMETRY_656    = 0x02,
    HDMI_COLORIMETRY_1120   = 0x03,
    HDMI_COLORIMETRY_SMPTE  = 0x04,
    HDMI_COLORIMETRY_XVYCC601 = 0x05,
    HDMI_COLORIMETRY_XVYCC709 = 0x06
};

enum HDMI_VIDEO_FORMAT {
    HDMI_NO_ADD_FORMAT,
    HDMI_4Kx2K_FORMAT,
    HDMI_3D_FORMAT
};

enum HDMI_4Kx2K_VIC {
    HDMI_4Kx2K_30HZ = 0x01,
    HDMI_4Kx2K_25HZ,
    HDMI_4Kx2K_24HZ,
    HDMI_4Kx2K_24HZ_SMPTE
};

enum HDMI_3D_STRUCTURE {
    HDMI_FRAME_PACKING,
    HDMI_FIELD_ALTERNATIVE,
    HDMI_LINE_ALTERNATIVE,
    HDMI_SIDE_BY_SIDE_FULL,
    L_DEPTH,
    L_DEPTH_GRAPHICS,
    SIDE_BY_SIDE_HALF = 8
};


struct dvin_config {
    enum DVIN_CS_MODE cs_mode;
    enum DVIN_BW_MODE bw_mode;
    enum DVIN_SQ_MODE sq_mode;
    enum DVIN_DR_MODE dr_mode;
    enum DVIN_SY_MODE sy_mode;
};

struct videotiming {
    uint8_t  polarity;
    uint16_t htotal;
    uint16_t vtotal;
    uint16_t hactive;
    uint16_t vactive;
    uint16_t pixclk;     /*10000hz*/
    uint16_t vfreq;      /*0.01hz*/
    uint16_t hoffset;    /* h sync start to h active*/
    uint16_t voffset;    /* v sync start to v active*/
    uint16_t hsyncwidth;
    uint16_t vsyncwidth;
};

struct hdmi_config {
    uint8_t hdmi_flag;				// 0 = dvi out;  1 = hdmi out
    uint8_t vic;				// reference to CEA-861 VIC
    uint16_t video_clk;			// TMDS video clk, uint 10000Hz
    enum HDMI_CLK_REPEAT clk_rpt;		// X2CLK = 480i/576i, others = X1CLK
    enum HDMI_SCAN_INFO scan_info;
    enum HDMI_ASPECT_RATIO aspect_ratio;
    enum HDMI_CS color_space;
    enum HDMI_COLOR_DEPTH color_depth;
    enum HDMI_COLORIMETRY colorimetry;	// IT601 = 480i/576i/480p/576p, ohters = IT709
    enum HDMI_VIDEO_FORMAT video_format;
    enum HDMI_4Kx2K_VIC h4Kx2K_vic;
    enum HDMI_3D_STRUCTURE h3D_structure;
    enum HDMI_AUDIO_MODE audio_mode;
    enum HDMI_AUDIO_RATE audio_rate;
    enum HDMI_AUDIO_LENGTH audio_bits;
    enum HDMI_AUDIO_CHN audio_channels;
    uint8_t audio_speaker_locations;		// 0~255, refer to CEA-861 audio infoframe, BYTE4
};

// *********************************************************************************************************************
typedef struct {
    int i2c_fd;
    uint8_t tx_channel;
    bool powered;
    
    // Configuration
    enum DVIN_CS_MODE cs_mode;
    enum DVIN_BW_MODE bw_mode; 
    enum DVIN_SQ_MODE sq_mode;
    enum DVIN_DR_MODE dr_mode;
    enum DVIN_SY_MODE sy_mode;
    
    bool rgb;
    int spdif;
} ms7210_dev_t;

#define MS7210_I2C_ADDR 0x56

int ms7210_dvin_timing_config(ms7210_dev_t *dev, struct dvin_config *dc, struct videotiming *vt, enum HDMI_CLK_REPEAT *rpt);
void ms7210_hdmi_tx_output_config(ms7210_dev_t *dev, struct hdmi_config *hc);
void print_usage(const char *name);
int ms7210_init(ms7210_dev_t *dev);

#ifdef __cplusplus
}
#endif