/*
 * BCAT-08-Tester.xc
 *
 *  Created on: 15/03/2017
 *      Author: bCat
 */

#include <platform.h>
#include <stdio.h>
#include <xccompat.h>
#include <string.h>
#include <xs1.h>
#include <xscope.h>
#include "bcat_avb_controller.h"
#include "ethernet_board_conf.h"
#include "app_config.h"
#include "avb_conf.h"
#include "i2c.h"
#include "smi.h"
#include "spi.h"
#include "avb_ethernet.h"
#include "audio_i2s.h"
#include "gptp.h"
#include "media_clock_server.h"
#include "avb.h"
#include "avb_1722_1.h"
#include "avb_1722.h"
#include "avb_srp.h"
// eth_status check include
#include "ethernet_link_status.h"


/*
 *  SET variables, enums, structs, etc
*/

// Memory is little endian (least significant byte at lowest address).
// -->   Channels can be considered to be big endian (most significant byte sent first over the the channel).

// Big endian : MSB at lowest address, left to right
// Little endian : LSB at lowest address, left to right
// spi byte order is Big Endian

// 1.1 is first board, 1.2 second, etc
// firmware revision matches board minor version
// running for board 5
#define BOARD_VERSION 1.5
#define FIRMWARE_VERSION 0.9.5
// boolean for IDE console print
#define DEBUG_P 1

/***************************************************************
 *
 *      BCAT DEFINES, ADDRS
 *
 ***************************************************************/
// PCA 9555D
#define LED_ADDR_1 0x20 // U19 - 0100|0000 - hex 0010|0000
#define LED_ADDR_2 0x21 // U20 - 0100|1000 - hex 0010|0001
#define LED_ADDR_3 0x22 // U21 - 0100|0100 - hex 0010|0010
#define LED_ADDR_4 0x23 // U22 - 0100|1100 - hex 0010|0011
// device registers
#define LED_CONFIG_0 0x06 // config port 0
#define LED_CONFIG_1 0x07 // config port 1
#define LED_OUTPUT_0 0x02 // output port 0
#define LED_OUTPUT_1 0x03 // output port 1
#define LED_INPUT_0 0x00  // input port 0
#define LED_INPUT_1 0x01  // input port 1

#define BUT0_PRESS 0x9 // SW1/but0 pressed
#define BUT1_PRESS 0x5 // SW2/but1 pressed

// ADC working addrs
#define ADC_ADDR_A 0x10
#define ADC_ADDR_B 0x11
#define ADC_ADDR_C 0x10
#define ADC_ADDR_D 0x11


const unsigned BCAT_SAMPLE_RATE = 48000;
unsigned char led_data[2] = {0,0};
unsigned char adc_data[2] = {0,0};
unsigned char but0_state = 0;
unsigned char but1_state = 0;
unsigned char read_button_data[1] = {0};
enum avb_source_state_t avb_state;

/***************************************************************
 *
 *      BCAT PORT DEFINITIONS
 *
 ***************************************************************/
//port PORT_SPI_MISO     = on tile[0] : XS1_PORT_1A
//port PORT_SPI_SS       = on tile[0] : XS1_PORT_1B
//port PORT_SPI_CLK      = on tile[0] : XS1_PORT_1C
//port PORT_SPI_MOSI     = on tile[0] : XS1_PORT_1D
//port ADC_MCLK          = on tile[0] : XS1_PORT_1E;
//port ADC_SCLK          = on tile[0] : XS1_PORT_1F;
//port ADC_LRCLK         = on tile[0] : XS1_PORT_1G;
//port UART_FROM_HOST    = on tile[0] : XS1_PORT_1I;
//port UART_TO_HOST      = on tile[0] : XS1_PORT_1L;
//port ADC_SDOUT         = on tile[0] : XS1_PORT_4C;
//port ADC_SDIN          = on tile[0] : XS1_PORT_4D;

//port PORT_ETH_RXCLK    = on tile[1] : XS1_PORT_1A;
//port PORT_ETH_RXERR    = on tile[1] : XS1_PORT_1B;
//port PORT_ETH_TXCLK    = on tile[1] : XS1_PORT_1C;
//port PORT_ETH_RXDV     = on tile[1] : XS1_PORT_1D;
//port PORT_ETH_TXEN     = on tile[1] : XS1_PORT_1E;
//port PORT_ETH_TXER     = on tile[1] : XS1_PORT_1F;
//port PORT_ETH_MDIO     = on tile[1] : XS1_PORT_1G;
//port PORT_ETH_MDC      = on tile[1] : XS1_PORT_1J;
//port PORT_ETH_RXD      = on tile[1] : XS1_PORT_4C;
//port PORT_ETH_TXD      = on tile[1] : XS1_PORT_4D;

//port MIDI_OUT          = on tile[1] : XS1_PORT_4A;
//port MIDI_IN           = on tile[1] : XS1_PORT_4B;
//port GSYNC_SEL         = on tile[1] : XS1_PORT_4E;

//port LED_nINT          = on tile[1] : XS1_PORT_1H;
//port LED_SDA           = on tile[1] : XS1_PORT_1M;
//port LED_SCL           = on tile[1] : XS1_PORT_1P;

//port AB_SDA            = on tile[1] : XS1_PORT_1K;
//port CD_SDA            = on tile[1] : XS1_PORT_1L;
//port AB_SCL            = on tile[1] : XS1_PORT_1N;
//port CD_SCL            = on tile[1] : XS1_PORT_1O;

//port NOT_USED          = on tile[1] : XS1_PORT_1I;


/***************************************************************
 *
 *      BCAT STRUCTS, ARRAYS
 *
 ***************************************************************/
on tile[0] : otp_ports_t otp_ports0 = OTP_PORTS_INITIALIZER;

//TODO
// this pin does not exist :: PLL_SYNC_OUT, remapped UART_FROM_HOST (XS1_PORT_1I)
on tile[0]: out buffered port:32 p_fs[1] = {
        XS1_PORT_1I
};
on tile[1] :
   struct r_i2c LED_i2cPorts = {
           XS1_PORT_1P,
           XS1_PORT_1M
   };

on tile[1] :
   struct r_i2c AB_i2cPorts = {
           XS1_PORT_1N,
           XS1_PORT_1K
   };

on tile[1] :
    struct r_i2c CD_i2cPorts = {
            XS1_PORT_1O,
            XS1_PORT_1L
    };

// conflicts with eth.reset on Eval_board: both set to X1D23 (1H)
// on schematic is currently via LED_ADDR_4 0x23 (U22, PCA9555D, pin 4, IOA0)
// LED_i2c_init() sets eth_rst low,
// reset_interface is an int for a port
// ETHERNET_DEFAULT_RESET_INTERFACE_INIT works -> == 0

// module_etherent/src/common/ethernet_phy_reset.h requires:
// int ethernet_reset_interface_t
// eth_phy_reset(ethernet_reset_interface_t eth_rst) {}
// *may* require it to be a port?

// typdef int eth_rst = 0x10a00 ( equal to XS1_PORT_1I )
// ethernet_reset_interface_t eth_rst_bcat = XS1_PORT_1I;
// warning over braces around scalar initialiser ETHERNET_RESET_INTERFACE
// eth_rst = 0 compiles
//
// 0 is not a valid port number error
// try PORT_ETH_RSTN

on tile[1] : avb_ethernet_ports_t avb_ethernet_ports = {
        on tile[1]: OTP_PORTS_INITIALIZER,
        ETHERNET_SMI_INIT,
        ETHERNET_MII_INIT_full,
        ETHERNET_DEFAULT_RESET_INTERFACE_INIT
};


// flash spi ports, XS1_CLKBLK_1 derived from Eval Board
on tile[0]: fl_spi_ports spi_ports = {
        PORT_SPI_MISO,
        PORT_SPI_SS,
        PORT_SPI_CLK,
        PORT_SPI_MOSI,
        SPI_CLKBLK
};


// i2s_ports_t constructor from avb_audio/audio_i2s.xc
on tile[0]: i2s_ports_t i2s_ports = {
  I2S_MCLK, // mclk, clkblk_3
  I2S_BCLK, // bclk, clkblk_4
  XS1_PORT_1E, // p_mclk, must be 1 bit IN port, this port to clk I2S_MCLK
  XS1_PORT_1F, // p_bclk, must be 1 bit OUT port, this port from clk I2S_BCLK
  XS1_PORT_1G // p_lrclk, 1 bit OUT port
};
// configure_clock_src(i2s.mclk, i2s.p_mclk);
// configure_clock_src(i2s.bclk, i2s.p_bclk);
// bclk == bit clock line, continuous serial clock (a.k.a. SCLK, SCK)
// lrclk == word clock line, word select left & right channel
// mclk == master clock used by audio codec


// i2s_master constructor calls start_port(p_aud_din[i]) etc, from avb_audio/audio_i2s.xc
// AVB_NUM_MEDIA_INPUTS == AVB_BCAT_NUM_CHANNELS == (L,R @ ADC1), i2s input channels
// AVB_NUM_SOURCES == avb sources ~ streams of multiple channels
// AVB_NUM_TALKER_UNITS == tasks running talkers
// what happens when 4 bit wide port data is input where 1 bit is expected and buffered?
// access the pin SDIN_A , X0D16 : 4D0 via lib_gpio?
#if AVB_BCAT_ENABLE_TALKER
    on tile[0]: in buffered port:32 p_aud_din[1] = {XS1_PORT_4D};
    media_input_fifo_data_t ififo_data[AVB_NUM_MEDIA_INPUTS];
    media_input_fifo_t ififos[AVB_NUM_MEDIA_INPUTS];
#else
    #define p_aud_din null
    #define ififos null
#endif

#if AVB_BCAT_ENABLE_LISTENER
    on tile[0]: out buffered port:32 p_aud_dout[1] = {XS1_PORT_4C};
    media_output_fifo_data_t ofifo_data[AVB_NUM_MEDIA_OUTPUTS];
    media_output_fifo_t ofifos[AVB_NUM_MEDIA_OUTPUTS];
#else
    #define p_aud_dout null
    #define ofifos null
#endif

/***************************************************************
 *
 *      BCAT CHANNELS
 *
 ***************************************************************/
enum mac_rx_chans {
    MAC_RX_TO_MEDIA_CLOCK = 0,
    #if AVB_BCAT_ENABLE_LISTENER
        MAC_RX_TO_LISTENER,
    #endif
    MAC_RX_TO_SRP,
    MAC_RX_TO_1722_1,
    NUM_MAC_RX_CHANS
};

enum mac_tx_chans {
    MAC_TX_TO_MEDIA_CLOCK = 0,
    #if AVB_BCAT_ENABLE_TALKER
        MAC_TX_TO_TALKER,
    #endif
    MAC_TX_TO_SRP,
    MAC_TX_TO_1722_1,
    MAC_TX_TO_AVB_MANAGER,
    NUM_MAC_TX_CHANS
};

enum ptp_chans {
    PTP_TO_AVB_MANAGER = 0,
    #if AVB_BCAT_ENABLE_TALKER
        PTP_TO_TALKER,
    #endif
    PTP_TO_1722_1,
    NUM_PTP_CHANS
};

enum avb_manager_chans {
    AVB_MANAGER_TO_SRP = 0,
    AVB_MANAGER_TO_1722_1,
    AVB_MANAGER_TO_BCAT,
    NUM_AVB_MANAGER_CHANS
};

/***************************************************************
 *
 *      UTILITIES
 *
 ***************************************************************/
void printBits(char byte) {
    unsigned char mask = 1;
    unsigned char bits[8];
    if (DEBUG_P) printf("\nbits: ");
    for (int i = 0; i < 8; i++) {
        bits[i] = (byte & (mask << i)) != 0;
        if (DEBUG_P) printf("%d", bits[i]);
    }
    if (DEBUG_P) printf("\n");
}

void printHex(char *message, unsigned char length) {
    if (DEBUG_P) printf("hex: ");
    for (int i = 0; i < length; i++) {
        if (DEBUG_P) printf("%X", message[i]);
    }
    if (DEBUG_P) printf("\n");
}

/*
// check MAC address, serial function
void OTP_query() {
    unsigned char mac_address[6];
    if (otp_board_info_get_mac(otp_ports1, 0, mac_address) == 0) {
        if (DEBUG_P) printf("no mac address found in OTP\n");
    }
    else {
        if (DEBUG_P) printf("found mac:\n");
        printHex(mac_address, 6);
    }
    if (DEBUG_P) printf("get serial:\n");
    unsigned int serial;
    otp_board_info_get_serial(otp_ports1, serial);
    if (DEBUG_P) printf("serial: %d\n", serial);
}
*/

unsigned int ETH_status() {
    // get status int from LAN8710a
    // reads via MDIO/MDC as up or down.
    //
    unsigned int status_report;
    status_report = ethernet_get_link_status(0);
    if (DEBUG_P) printf("eth_status: %d\n", status_report);
    //
    return status_report;
}

/*
timer t;
unsigned int time;
#define DELAY 5000000

unsigned int init_delay() {
    // timer delayed start for eth:
    t :> time;
    for (int i = 0; i < 2; i++) {
        select {
            case t when timerafter(time) :> void:
                time += XS1_TIMER_MHZ * DELAY;
                if (i != 0) {
                    printf("mark 5 seconds.\n");
                    return 1;
                }
                break;
        }
    }
    return 0;
}
*/

/***************************************************************
 *
 *      BCAT FUNCTIONS
 *
 **************************************************************/
void eth_rst_toggle(unsigned int toggle_on) {
    if (DEBUG_P) printf("eth_rst_toggle(): %d.\n", toggle_on);
    // writes will automatically use port A then goto port B, so must include their values
    unsigned char num_bytes = 2;
    int write_result;

    // eth_nRST pin0 drive low causes hardware reset
    // port A pins 7,6,5,4 = LEDs; 3,2,1,0 = gain_selH, phant_en7, analog_nRST, eth_nRST
    // 1111 1011 - data to port A = 7,6,5,4,3 to high; 2 to low; 1,0 to high

    // port B pins 7,6,5,4 = but1, but0, phant_en6, gain_selG; 3,2,1,0 = LEDs
    // 0001 1111 - data to port B = 7,6,5,4 = ignored; 3,2,1,0 = high; n.b. ignored as per datasheet

    if (toggle_on) {
        // on
        led_data[0] = 0xFB;
    }
    else {
        // off
        led_data[0] = 0xFA;
    }
    led_data[1] = 0x1F;
    write_result = i2c_master_write_reg(LED_ADDR_4, LED_OUTPUT_0, led_data, num_bytes, LED_i2cPorts);
    if (write_result == 0) {
        if (DEBUG_P) printf("LED_ADDR_4 write_result: %D ", write_result);
       printHex(led_data, num_bytes);
    }
}

/*
void I2C_ADC_sdata() {
    printf("I2C_ADC_sdata().\n");
    // parallel usage rules apply
    unsigned int empty;
    empty = media_input_fifo_empty(ififos[0]);
    unsigned int size;
    // this can get zeroed by media_input_fifo.c: media_input_fifo_flush()
    // when:  if (active_fifos) else (flush)
    size = ififo_data[0].sampleCountInPacket;
    unsigned char clockM;
    i2s_ports.p_mclk :> clockM;
    printf("sdata is empty: %d, size: %d, mclk: %d\n", empty, size, clockM);
}
*/

/*
CS4272 adc
registers:
0x01 mode control 1
0x02 DAC control
0x03 DAC volume & mixing control
0x04 DAC ch.A volume control
0x05 DAC ch.B volume control
0x06 ADC control
0x07 mode control 2 (cpe)
0x08 chip ID
 */
//TODO
void I2C_ADC_init() {
    if (DEBUG_P) printf("I2C_ADC_init() 15:14.\n");
    // Control Port mode
    unsigned char read_data[2] = {0,0};
    unsigned char reg_mode_ctrl = 0x01;
    //unsigned char reg_dac_ctrl = 0x02;
    //unsigned char reg_mix_ctrl = 0x03;
    //unsigned char reg_dacA_vol = 0x04;
    //unsigned char reg_dacB_vol = 0x05;
    //unsigned char reg_adc_ctrl = 0x06;
    unsigned char reg_addr_cpe = 0x07;
    //unsigned char reg_addr_chip_id = 0x08;
    unsigned char num_bytes = 1;

    //ADC_ADDR 0010000
    // 7-5=reserved, 4=LOOP, 3=MUTECA=B, 2=FREEZE, 1=CPEN, 0=PDN
    adc_data[0] = 0xAA; // 1010 1010 LOOP off
    //adc_data[0] = 0xBA; // 1011 1010 LOOP on
// WRITE
    int write_result; // will return 1 = OK, 0 = error
    write_result = i2c_master_write_reg(ADC_ADDR_A, reg_addr_cpe, adc_data, num_bytes, AB_i2cPorts);
    //i2c_master_write_reg(ADC_ADDR_B, reg_addr_cpe, adc_data, num_bytes, AB_i2cPorts);
    if (write_result == 0) {
        if (DEBUG_P) printf("write_result: %D ", write_result);
        printHex(adc_data, num_bytes);
    }

    // in MASTER mode LRLCK, SCLK are outputs generated on ADC chip
    //adc_data[0] = 0x08; // 0000 1000 set to adc ctrl master - do not need master for LOOP
    adc_data[0] = 0x00; // 0000 0000 set to adc ctrl slave, default
    write_result = i2c_master_write_reg(ADC_ADDR_A, reg_mode_ctrl, adc_data, num_bytes, AB_i2cPorts);
    if (write_result == 0) {
        if (DEBUG_P) printf("write_result: %D ", write_result);
        printHex(adc_data, num_bytes);
    }

// READ REGs LOOP
    // confirm that ports are set to send audio
    read_data[0] = 0x0;
    int read_result; // will return 1 = OK, 0 = error
    for (int i = 0; i < 8; i++) {
        read_result = i2c_master_read_reg(ADC_ADDR_A, reg_mode_ctrl + i, read_data, num_bytes, AB_i2cPorts);
        if (DEBUG_P) printf("A reg %d read_result: %D ", i + 1, read_result);
        printHex(read_data, num_bytes);
    }
}

void LED_i2c_buttons() {
    // read input port 1 register for button states
    i2c_master_read_reg(LED_ADDR_4, LED_INPUT_1, read_button_data, 1, LED_i2cPorts);
    // SW1/but0 press&hold == hex: 9|E -> 1001|~
    // SW2/but1 press&hold == hex: 5|A -> 0101|~
    if ((read_button_data[0] >> 4) == BUT0_PRESS) {
        //if (DEBUG_P) printf("but0_press\n");
        //but0_state = !but0_state;
        //red
        but0_state = 1;
        but1_state = 0;
        //eth_rst_toggle(0);
        ETH_status();
    }
    else if ((read_button_data[0] >> 4) == BUT1_PRESS) {
        //if (DEBUG_P) printf("but1_press\n");
        //but1_state = !but1_state;
        //green
        but0_state = 0;
        but1_state = 1;
        //eth_rst_toggle(1);
        ETH_status();
    }
    else {
        // amber
    }
}

void I2C_LED_gen(unsigned char *gen_data) {
    // two LEDs near DC power input, on LED_ADDR_1
    // change r/g LED based upon switch press&hold
    short gen_LEDs = 0x0000;

    if (but0_state) {
        gen_LEDs = 0x05; // red, 0000 0101
    }
    else if (but1_state) {
        gen_LEDs = 0x0A; // green, 0000 1010
    }
    else {
        // amber
    }
    // byte 1 = lower gen_led + top main led L/R pair
    // byte 2 = upper gen_led + 2nd main LED pair L/R
    gen_data[0] = gen_data[0] & 0x0C; // 0000 1100
    gen_data[1] = gen_data[1] & 0x30; // 0011 0000
    gen_data[0] = gen_data[0] | ((gen_LEDs >> 0) & 0x03);
    gen_data[1] = gen_data[1] | (((gen_LEDs >> 2) & 0x03) << 6);
    // gen_data[0] = 0x01; // red 0000 0001
    // gen_data[1] = 0x40; // red 0100 0000
    // gen_data[0] = 0x02; // green 0000 0010
    // gen_data[1] = 0x80; // green 1000 0000
}

// test cycle for each main LEDs
void I2C_LED_test() {
    if (DEBUG_P) printf("LED_i2c_test().\n");
    //short aLEDs = 0x0003;
    short gLEDs = 0x0001;
    short rLEDs = 0x0000;
    unsigned char state = 0;
    int i = 0;

    int led_port_enum = LED_ADDR_1;
    unsigned char shifter_hi;
    unsigned char shifter_lo;
    unsigned char num_bytes = 2;

    ETH_status();

//    while(1) {
        // needs to be 16 == num of LEDs
        // need an on input change, not while loop constant poll...
        LED_i2c_buttons();
        if (i < 16) {
            i++;
            if (state == 0) {
                // off
                gLEDs = 0xFFFF; // off
                rLEDs = (rLEDs << 1); // on
            }
            else {
                // on
                rLEDs = 0xFFFF; // off
                gLEDs = (gLEDs << 1); // on
            }
            //set these
            shifter_hi = 15; // 15, 13, 11, 9
            shifter_lo = 7;  // 7, 5, 3, 1

            for (int j = 0; j < 4; j++) {
                // first read present state of outputs into led_data
                i2c_master_read_reg(led_port_enum + j, LED_OUTPUT_0, led_data, num_bytes, LED_i2cPorts);

                // if j == LED_ADDR_1
                // called 4 times per each i loop
                if (j == 0) {
                    I2C_LED_gen(led_data);
                }

                // LED chart (updated 30/03)
                //chip  U19     U20     U21     U22     ::    U19     U20     U21     U22
                //                shifter_hi            ::              shifter_lo
                //RED   A1 B1   A1 B1   A1 B1   A1 B1   ::    A3 B3   A3 B3   A3 B3   A3 B3
                //I2C   0x20    0x21    0x22    0x23    ::    0x20    0x21    0x22    0x23
                //Port  A5 B2   A5 B2   A5 B2   A5 B2   ::    A7 B0   A7 B0   A7 B0   A7 B0
                //BIT   15 14   13 12   11 10   09 08   ::    07 06   05 04   03 02   01 00
                //LED   1A 3A   1B 3B   1C 3C   1D 3D   ::    4D 2D   4C 2C   4B 2B   4A 2A

                //GRN   A0 B0   A0 B0   A0 B0   A0 B0   ::    A2 B2   A2 B2   A2 B2   A2 B2
                //I2C   0x20    0x21    0x22    0x23    ::    0x20    0x21    0x22    0x23
                //Port  A4 B3   A4 B3   A4 B3   A4 B3   ::    A6 B1   A6 B1   A6 B1   A6 B1
                //BIT   15 14   13 12   11 10   09 08   ::    07 06   05 04   03 02   01 00
                //LED   1A 3A   1B 3B   1C 3C   1D 3D   ::    4D 2D   4C 2C   4B 2B   4A 2A

                // schematic :
                // A_LED0, A_LED2 = green ;  A_LED1, A_LED3 = red ; same for B_LEDn

                //i2c_master_read_reg(led_port_enum + j, LED_OUTPUT_0, led_data, num_bytes, LED_i2cPorts);
                led_data[0] = led_data[0] & 0x0F; // & 0000 1111 for r/gLEDs
                led_data[1] = led_data[1] & 0xF0; // & 1111 0000 for r/gLEDs

                //led_data OR (gLEDs right shift 15 bitwise AND 0x01) left shift 4
                //check the green
                led_data[0] = led_data[0] | ((gLEDs >> shifter_hi) & 0x01) << 4;        // bit 15, port A4
                led_data[1] = led_data[1] | ((gLEDs >> shifter_hi - 1) & 0x01) << 3;    // bit 14, port B3
                // check the red
                led_data[0] = led_data[0] | ((rLEDs >> shifter_hi) & 0x01) << 5;        // bit 15, port A5
                led_data[1] = led_data[1] | ((rLEDs >> shifter_hi - 1) & 0x01) << 2;    // bit 14, port B2

                // check the green
                led_data[0] = led_data[0] | ((gLEDs >> shifter_lo) & 0x01) << 6;        // bit 7, port A6
                led_data[1] = led_data[1] | ((gLEDs >> shifter_lo - 1) & 0x01) << 1;    // bit 6, port B1
                // check the red
                led_data[0] = led_data[0] | ((rLEDs >> shifter_lo) & 0x01) << 7;        // bit 7, port A7
                led_data[1] = led_data[1] | ((rLEDs >> shifter_lo - 1) & 0x01) << 0;    // bit 6, port B0

                shifter_hi -= 2;
                shifter_lo -= 2;
                i2c_master_write_reg(led_port_enum + j, LED_OUTPUT_0, led_data, num_bytes, LED_i2cPorts);
            }
        }
        else {
            i = 0;
            // flip state
            state = !state;
        }
        delay_milliseconds(50);
//    }
}

void I2C_init() {
    if (DEBUG_P) printf("I2C_init().\n");
    int led_port_enum = LED_ADDR_1;
    unsigned char num_bytes = 2;
    //unsigned char read_data[2] = {0,0};
    int i;
    // (port A = chip port 0, port B = chip port 1)
    // PCA9555 pairs the registers to byte 0 and byte 1
    // data to port MSB = 7,6,5,4,3,2,1,0

    // Configure LED i2c chips 1(U19), 2(U20), 3(U21) for outputs drive outputs High
    led_data[0] = 0x00; // 0000 0000 - port A  = output
    led_data[1] = 0x00; // 0000 0000 - port B  = output
    int write_result;
    for (i = 0; i < 3; i++) {
        i2c_master_write_reg(led_port_enum + i, LED_CONFIG_0, led_data, num_bytes, LED_i2cPorts);
    }

    // Drive outputs High except for Phantom Power Enable to low
    led_data[0] = 0xFB; // 1111 1011 - data to port A = pin 2 to low
    led_data[1] = 0xDF; // 1101 1111 - data to port B = pin 5 to low
    for (i = 0; i < 3; i++) {
        i2c_master_write_reg(led_port_enum + i, LED_OUTPUT_0, led_data, num_bytes, LED_i2cPorts);
    }

    // Configure LED i2c chip 4(U22) to be all outputs
    // except for Port B pins 6,7 which are the buttons
    // on board: but0=SW1(left), but1=SW2(right)

    // port A pins all output
    // 0000 0000 - port A = output
    led_data[0] = 0x00;

    //port B pins 7,6,5,4 = but1, but0, phant_en6, gain_selG; 3,2,1,0 = LEDs
    // 1111 0000 - port B = 7,6,5,4 input; 3,2,1,0 output
    led_data[1] = 0xF0;

    write_result = i2c_master_write_reg(LED_ADDR_4, LED_CONFIG_0, led_data, num_bytes, LED_i2cPorts);
    if (write_result == 0) {
       printf("LED_CONFIG_0 write_result: %D ", write_result);
       printHex(led_data, num_bytes);
    }

    // eth_nRST pin0 drive low causes hardware reset
    // port A pins 7,6,5,4 = LEDs; 3,2,1,0 = gain_selH, phant_en7, analog_nRST, eth_nRST
    // 1111 1011 - data to port A = 7,6,5,4,3 to high; 2 to low; 1,0 to high
    led_data[0] = 0xFB;

    // port B pins 7,6,5,4 = but1, but0, phant_en6, gain_selG; 3,2,1,0 = LEDs
    // 0001 1111 - data to port B = 7,6,5,4 = ignored; 3,2,1,0 = high; n.b. ignored as per datasheet
    led_data[1] = 0x1F;

    write_result = i2c_master_write_reg(LED_ADDR_4, LED_OUTPUT_0, led_data, num_bytes, LED_i2cPorts);
    if (write_result == 0) {
        if (DEBUG_P) printf("LED_OUTPUT_0 write_result: %D ", write_result);
       printHex(led_data, num_bytes);
    }

    /*
    // LED 4, PORT A has eth_rst on bit 0
    read_data[0] = 0x0;
    read_data[1] = 0x0;
    int read_result; // will return 1 = OK, 0 = error
    // led_output_0 read is same as write values
    // led_config_0 read is same as write values
    //for (int i = 0; i < 8; i++) {
        read_result = i2c_master_read_reg(LED_ADDR_4, LED_OUTPUT_0, read_data, num_bytes, LED_i2cPorts);
        printf("LED_4 output read_result: %D ", read_result);
        printHex(read_data, num_bytes);
    //}
    */
    I2C_ADC_init();

    I2C_LED_test();
}

/***************************************************************
 *
 *      PAR FUNCTIONS
 *
 **************************************************************/
/*
[[distributable]] void audio_hardware_setup(void) {
    if (DEBUG_P) printf("audio hardware setup\n");
    //TODO
    // this does i2c reg writes to non-existent CP2300 chip:
    // normally, sets up clock multiplier rates based on PLL_SYNC_OUT ( media_clock_server(p_fs) )
    // as well as CP2300 sending out ADC/DAC_MCLK rates, XCORE_MCLK (in port PORT_MCLK)
    audio_clock_CS2300CP_init(AB_i2cPorts, MASTER_TO_WORDCLOCK_RATIO);
    //audio_clock_CS2300CP_init(CD_i2cPorts, MASTER_TO_WORDCLOCK_RATIO);
    //I2C_ADC_init();

    while (1) {
        select {
            //
        }
    }
}
*/

// AVB control task using 1722_1
[[combinable]]
void application_task(client interface avb_interface avb, server interface avb_1722_1_control_callbacks i_1722_1_callbacks) {
    if (DEBUG_P) printf("applicationTask\n");
    I2C_init();

    unsigned char aem_identify_control_value = 0;

    // Initialize the media clock
    avb.set_device_media_clock_type(0, DEVICE_MEDIA_CLOCK_INPUT_STREAM_DERIVED);
    avb.set_device_media_clock_rate(0, BCAT_SAMPLE_RATE);
    avb.set_device_media_clock_state(0, DEVICE_MEDIA_CLOCK_STATE_ENABLED);
    #if AVB_BCAT_ENABLE_TALKER
        for (int j = 0; j < AVB_NUM_SOURCES; j++) {
            const int channels_per_stream = AVB_NUM_MEDIA_INPUTS / AVB_NUM_SOURCES;
            int map[AVB_NUM_MEDIA_INPUTS / AVB_NUM_SOURCES];
            for (int i = 0; i < channels_per_stream; i++) {
                map[i] = j ? j * (channels_per_stream) + i : j + i;
            }
            avb.set_source_map(j, map, channels_per_stream);
            avb.set_source_format(j, AVB_SOURCE_FORMAT_MBLA_24BIT, BCAT_SAMPLE_RATE);
            avb.set_source_sync(j, 0);
            avb.set_source_channels(j, channels_per_stream);
        }
    #endif
    avb.set_sink_format(0, AVB_SOURCE_FORMAT_MBLA_24BIT, BCAT_SAMPLE_RATE);
/*
    // report on avb_source state
    unsigned int get_result; // boolean
    get_result = avb_get_source_state(avb, 0, avb_state);
    if (DEBUG_P) printf("get_result: %d, avb_state: %d\n", get_result, avb_state);
    // if get 0: AVB_SOURCE_STATE_DISABLED
*/
    if (DEBUG_P) printf("application_task while loop\n");

    while (1) {
        select {

            case i_1722_1_callbacks.get_control_value(
                        unsigned short control_index,
                        unsigned short &values_length,
                        unsigned char values[AEM_MAX_CONTROL_VALUES_LENGTH_BYTES]) -> unsigned char return_status: {

                if (DEBUG_P) printf("get control val\n");
                return_status = AECP_AEM_STATUS_NO_SUCH_DESCRIPTOR;
                switch (control_index) {
                    case DESCRIPTOR_INDEX_CONTROL_IDENTIFY:
                        values[0] = aem_identify_control_value;
                        values_length = 1;
                        return_status = AECP_AEM_STATUS_SUCCESS;
                        break;
                }
                break;
            }

            case i_1722_1_callbacks.set_control_value(
                        unsigned short control_index,
                        unsigned short values_length,
                        unsigned char values[AEM_MAX_CONTROL_VALUES_LENGTH_BYTES]) -> unsigned char return_status: {

                if (DEBUG_P) printf("set control val\n");
                return_status = AECP_AEM_STATUS_NO_SUCH_DESCRIPTOR;
                switch (control_index) {
                    case DESCRIPTOR_INDEX_CONTROL_IDENTIFY: {
                        if (values_length == 1) {
                            aem_identify_control_value = values[0];
                            // OSX: Audio MIDI setup util, network device browser, identify trips LED
                            // p_mute_led_remote <: (~0) & ~((int)aem_identify_control_value<<1);
                            if (aem_identify_control_value) {
                                debug_printf("IDENTIFY Ping\n");
                            }
                            return_status = AECP_AEM_STATUS_SUCCESS;
                        }
                        else {
                            return_status = AECP_AEM_STATUS_BAD_ARGUMENTS;
                        }
                        break;
                    }
                }
                break;
            }
        }
    }

}

/*
#define NUM_SAMPLES 64
unsigned int sine[NUM_SAMPLES] = {
        4000,4392,4780,5161,5531,5886,6222,6538,6828,7092,7326,7528,7696,7828,7923,7981,
        8000,7981,7923,7828,7696,7528,7326,7092,6828,6538,6222,5886,5531,5161,4780,4392,
        4000,3608,3220,2839,2469,2114,1778,1462,1172,908,674,472,304,172,77,19,
        0,19,77,172,304,472,674,908,1172,1462,1778,2114,2469,2839,3220,3608,
};

void audio_scope() {
    //int sample;
    while(1) {
        //p_aud_din[0] :> sample;
        //xscope_int(0, sample);
        // not audio, check the clocks
        for (unsigned int i = 0; i < NUM_SAMPLES; ++i) {
            xscope_int(TEST, sine[i]);
        }
    }
}
*/

int main(void) {
    // Ethernet channels
    chan c_mac_tx[NUM_MAC_TX_CHANS]; // chanend connection to the Ethernet TX server
    chan c_mac_rx[NUM_MAC_RX_CHANS];
    // PTP channels
    chan c_ptp[NUM_PTP_CHANS];

    // AVB unit control
    #if AVB_BCAT_ENABLE_TALKER
        chan c_talker_ctl[AVB_NUM_TALKER_UNITS];
    #else
        #define c_talker_ctl null
    #endif

    #if AVB_BCAT_ENABLE_LISTENER
        chan c_listener_ctl[AVB_NUM_LISTENER_UNITS];
        chan c_buf_ctl[AVB_NUM_LISTENER_UNITS];
    #else
        #define c_listener_ctl null
        #define c_buf_ctl null
    #endif

    // Media control
    chan c_media_ctl[AVB_NUM_MEDIA_UNITS];
    interface media_clock_if i_media_clock_ctl;

    interface avb_interface i_avb[NUM_AVB_MANAGER_CHANS];
    interface srp_interface i_srp;
    interface avb_1722_1_control_callbacks i_1722_1_callbacks;

    par {
        on tile[1]: avb_ethernet_server(avb_ethernet_ports,
                                           c_mac_rx,
                                           NUM_MAC_RX_CHANS,
                                           c_mac_tx,
                                           NUM_MAC_TX_CHANS);

        on tile[0]: media_clock_server(i_media_clock_ctl,
                                           null,
                                           c_buf_ctl,
                                           AVB_NUM_LISTENER_UNITS,
                                           p_fs,
                                           c_mac_rx[MAC_RX_TO_MEDIA_CLOCK],
                                           c_mac_tx[MAC_TX_TO_MEDIA_CLOCK],
                                           c_ptp,
                                           NUM_PTP_CHANS,
                                           PTP_GRANDMASTER_CAPABLE);

        // AVB - Audio
        on tile[0]: {
            #if AVB_BCAT_ENABLE_TALKER
                  init_media_input_fifos(ififos, ififo_data, AVB_NUM_MEDIA_INPUTS);
            #endif

            #if AVB_BCAT_ENABLE_LISTENER
                  init_media_output_fifos(ofifos, ofifo_data, AVB_NUM_MEDIA_OUTPUTS);
            #endif

            // i2s_ports struct on tile[0]
            i2s_master(i2s_ports,
                    p_aud_din,
                    AVB_NUM_MEDIA_INPUTS,
                    p_aud_dout,
                    AVB_NUM_MEDIA_OUTPUTS,
                    MASTER_TO_WORDCLOCK_RATIO,
                    ififos,
                    ofifos,
                    c_media_ctl[0],
                    0);
        }

        #if AVB_BCAT_ENABLE_TALKER
            // AVB Talker - must be on the same tile as the audio interface (fifos)
            on tile[0]: avb_1722_talker(c_ptp[PTP_TO_TALKER],
                                        c_mac_tx[MAC_TX_TO_TALKER],
                                        c_talker_ctl[0],
                                        AVB_NUM_SOURCES);
        #endif

        #if AVB_BCAT_ENABLE_LISTENER
            // AVB Listener
            on tile[0]: avb_1722_listener(c_mac_rx[MAC_RX_TO_LISTENER],
                                          c_buf_ctl[0],
                                          null,
                                          c_listener_ctl[0],
                                          AVB_NUM_SINKS);
        #endif

        on tile[1]: [[combine]] par {
            avb_manager(i_avb,
                    NUM_AVB_MANAGER_CHANS,
                    i_srp,
                    c_media_ctl,
                    c_listener_ctl,
                    c_talker_ctl,
                    c_mac_tx[MAC_TX_TO_AVB_MANAGER],
                    i_media_clock_ctl,
                    c_ptp[PTP_TO_AVB_MANAGER]);

            avb_srp_task(i_avb[AVB_MANAGER_TO_SRP],
                    i_srp,
                    c_mac_rx[MAC_RX_TO_SRP],
                    c_mac_tx[MAC_TX_TO_SRP]);
        }

        on tile[1]: application_task(i_avb[AVB_MANAGER_TO_BCAT], i_1722_1_callbacks);

        on tile[0]: avb_1722_1_maap_task(otp_ports0,
                    i_avb[AVB_MANAGER_TO_1722_1],
                    i_1722_1_callbacks,
                    null,
                    c_mac_rx[MAC_RX_TO_1722_1],
                    c_mac_tx[MAC_TX_TO_1722_1],
                    c_ptp[PTP_TO_1722_1]);

        // for tile[0] rem out avb_1722_1_maap_task above to use OTP_query, vice versa
        //on tile[0]: OTP_query();
    }
    return 0;
}
