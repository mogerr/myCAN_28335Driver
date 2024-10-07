// file encoding utf-8

#include "myCan/myCAN_28335Driver.h"
#include "myCan/myCAN_Config.h"

//include your headers
//...

// for TX Example
struct myCan_PKT TX_Example_PKT = {0x0A106115, MBX_DLC_3, {0x01, 0x1A, 0xF1}};
int i = 0;

void main()
{
    //your CPU init
    //...

    //myCAN device init
    myCan_Init();

    //myCAN config init
    myCan_P_Config();

    //myCAN interrupt config init
    myCan_P_Config_IT();

    while(1)
    {
        //myCAN poll handling 
        myCan_poll();
        
        // for TX Example 
        if(i >= 10000)
        {
            i = 0;
            myCan_MBX_pkt2MD(myCana, MBX_1, &TX_Example_PKT);
            myCan_MBX_Trans_withTimeOut(myCana, MBX_1, 200);
            // myCan_MBX_Trans(myCana, MBX_1);
            // myCan_MBX_Trans_withBlock(myCana, MBX_1);
        }
        else
        {
            i++;
        }
    }
}

// RX Example using interrupt, found in myCAN_Config.c file.
