// file encoding utf-8

#ifndef _MYCAN_CONFIG_H_
#define _MYCAN_CONFIG_H_

#include "myCan_28335Driver.h"

struct CAN_P{
    char                name[32];
    struct myCan       *myCan_x;
    enum MBX_N          mbx_x;
    enum MBX_DIR        mbx_dir;
    struct myCan_PKT    mbx_pkt;
    Uint8               isUpdateFlag;
};

void myCan_P_Config();
void myCan_P_Config_IT();

#pragma CODE_SECTION(Ecana_isr1, "ramfuncs");
__interrupt void Ecana_isr1(void);


#endif
