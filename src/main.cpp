#include "ms7210_wrap.h"

int main(int argc, char *argv[])
{
    ms7210_dev_t dev;
    char *i2c_dev = "/dev/i2c-7";
    int width = 1920;
    int height = 1080;
    int refresh = 60;
    int opt;

    struct option long_options[] = {
        {"device", required_argument, 0, 'd'},
        {"width", required_argument, 0, 'w'},
        {"height", required_argument, 0, 'h'},
        {"refresh", required_argument, 0, 'r'},
        {"color", required_argument, 0, 'c'}, 
        {"spdif", required_argument, 0, 's'},
        {"help", no_argument, 0, '?'},
        {0, 0, 0, 0}
    };

    while((opt = getopt_long(argc, argv, "d:w:h:r:c:s:?",
                            long_options, NULL)) != -1) {
        switch(opt) {
            case 'd':
                i2c_dev = optarg;
                break;
            case 'w':
                width = atoi(optarg);
                break;
            case 'h':
                height = atoi(optarg);
                break;
            case 'r':
                refresh = atoi(optarg);
                break;
            case 'c':
                if(strcmp(optarg, "rgb") == 0)
                    dev.cs_mode = DVIN_CS_MODE_RGB;
                else if(strcmp(optarg, "yuv444") == 0) 
                    dev.cs_mode = DVIN_CS_MODE_YUV444;
                else if(strcmp(optarg, "yuv422") == 0)
                    dev.cs_mode = DVIN_CS_MODE_YUV422;
                break;
            case 's':
                dev.spdif = atoi(optarg);
                break;
            case '?':
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return -1;
        }
    }

    printf("Configuring MS7210:\n");
    printf("I2C device: %s\n", i2c_dev);
    printf("Resolution: %dx%d@%d\n", width, height, refresh);

    // Open I2C device
    dev.i2c_fd = open(i2c_dev, O_RDWR);
    if(dev.i2c_fd < 0) {
        printf("Failed to open %s: %s\n", i2c_dev, strerror(errno));
        return -1;
    }

    // Initialize MS7210
    if(ms7210_init(&dev) < 0) {
        printf("MS7210 init failed\n");
        close(dev.i2c_fd);
        return -1;
    }

    struct videotiming vt = {
        .polarity = 0x07,
        .htotal = 2200,
        .vtotal = 1125,
        .hactive = 1920,
        .vactive = 1080,
        .pixclk = 14850,
        .vfreq = 6000,
        .hoffset = 192,
        .voffset = 41,
        .hsyncwidth = 44,
        .vsyncwidth = 5
    };

    struct hdmi_config hc = {
        .hdmi_flag = true,
        .vic = 0,
        .clk_rpt = HDMI_X1CLK,
        .scan_info = HDMI_OVERSCAN,
        .aspect_ratio = HDMI_16X9,
        .color_space = HDMI_RGB,
        .color_depth = HDMI_COLOR_DEPTH_8BIT,
        .colorimetry = HDMI_COLORIMETRY_709,
        .video_format = HDMI_NO_ADD_FORMAT,
        .h4Kx2K_vic = HDMI_4Kx2K_30HZ,
        .h3D_structure = HDMI_FRAME_PACKING,
        .audio_mode = HDMI_AUD_MODE_AUDIO_SAMPLE,
        .audio_rate = HDMI_AUD_RATE_48K,
        .audio_bits = HDMI_AUD_LENGTH_16BITS,
        .audio_channels = HDMI_AUD_2CH,
        .audio_speaker_locations = 0
    };

    struct dvin_config dc = {
        .cs_mode = dev.cs_mode,
        .bw_mode = dev.bw_mode,
        .sq_mode = dev.sq_mode,
        .dr_mode = dev.dr_mode,
        .sy_mode = dev.sy_mode
    };

    // Configure timing
    int clkx2 = ms7210_dvin_timing_config(&dev, &dc, &vt, &hc.clk_rpt);
    hc.video_clk = vt.pixclk;

    // Configure output 
    ms7210_hdmi_tx_output_config(&dev, &hc);

    printf("MS7210 configured successfully\n");

    close(dev.i2c_fd);
    return 0;
}