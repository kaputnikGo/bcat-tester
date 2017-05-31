#ifndef __ethernet_board_conf_h__
#define __ethernet_board_conf_h__

#ifdef __ethernet_conf_h_exists_
#include "ethernet_conf.h"
#endif

#include "ethernet_conf_derived.h"

#define ETHERNET_DEFAULT_TILE tile[1]
// 8710a default addr 000b, checked with special modes register as being 0x0
#define ETHERNET_DEFAULT_PHY_ADDRESS 0x0

#define ETHERNET_CLKBLK_0 on ETHERNET_DEFAULT_TILE: XS1_CLKBLK_1
#define ETHERNET_CLKBLK_1 on ETHERNET_DEFAULT_TILE: XS1_CLKBLK_2

// try force ethernet_phy_reset.xc with unconnected port_eth_rstn
// ethernet_reset_interface_t eth_rst is either
//  an out port with :
//  ETHERNET_DEFAULT_RESET_INTERFACE_INIT PORT_ETH_RSTN or
//  an int with:
//  ETHERNET_DEFAULT_RESET_INTERFACE_INIT 0
// conflicts with eth.reset on Eval_board: both set to X1D23 (1H)
// on schematic is currently via LED_ADDR_4 0x23 (U22, PCA9555D, pin 4, IOA0)
// LED_i2c_init() sets eth_rst low,
// tries to use temp assign <Port Location="XS1_PORT_1I" Name="ETH_RST"/>
// not using ETHERNET_DEFAULT_RESET_INTERFACE_INIT
//#define ETHERNET_RESET_INTERFACE { ETH_RST }

#define ETHERNET_MII_INIT_full {\
    ETHERNET_CLKBLK_0,          \
    ETHERNET_CLKBLK_1,          \
    XS1_PORT_1A,                \
    XS1_PORT_1B,                \
    XS1_PORT_4C,                \
    XS1_PORT_1D,                \
    XS1_PORT_1C,                \
    XS1_PORT_1E,                \
    XS1_PORT_4D                 \
}

#define ETHERNET_MII_INIT ADD_SUFFIX(ETHERNET_MII_INIT,ETHERNET_DEFAULT_IMPLEMENTATION)

#define ETHERNET_SMI_INIT {\
       ETHERNET_DEFAULT_PHY_ADDRESS,    \
       XS1_PORT_1G,                     \
       XS1_PORT_1J                      \
}

#endif // __ethernet_board_conf_h__
