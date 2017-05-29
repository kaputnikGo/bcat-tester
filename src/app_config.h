#ifndef __app_config_h__
#define __app_config_h__

/***** APPLICATION CONFIGURATION ******/

// Global switches
#define AVB_BCAT_ENABLE_TALKER 1
#define AVB_BCAT_ENABLE_LISTENER 1

/** Number of input/output audio channels in the demo application
  * For simplicity, input and output is identical in size but can be configured
  * differently in ``avb_conf.h``. */
#define AVB_BCAT_NUM_CHANNELS 4
// dropped from num_channels 8 as SDOUT,SDIN are using a single (4 bit) port.

#define PLL_TYPE_CS2300         1

#define I2C_COMBINE_SCL_SDA     0

/** Add sine wave synthesis from channels ``I2S_SYNTH_FROM*2`` upwards in the I2S component */
/*
 * Finally, the XR-AVB-LC-BRD has eight digital inputs but only two of them are connected to an ADC on board.
 * For this demo another setting is added, which causes the I2S component to ignore the input on every stereo
 * pair but the first (ch 1&2), and instead adds synthesized sine waves to these inputs (chs 3-8):
 */
#define I2S_SYNTH_FROM 1

// CLOCKS
#define SPI_CLKBLK on tile[0] : XS1_CLKBLK_1
#define I2S_MCLK on tile[0] : XS1_CLKBLK_3
#define I2S_BCLK on tile[0] : XS1_CLKBLK_4
// eth clocks defined on ethernet_board_conf.h

#endif // __app_config_h__
