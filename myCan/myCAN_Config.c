// file encoding utf-8

#include "myCan_28335Driver.h"
#include "myCan_Config.h"

//////////////////////////////////////////////////////////
inline void myCan_P_set(struct CAN_P *p){
    myCan_MBX_Disable   (p->myCan_x, p->mbx_x);
    myCan_MBX_DIR_change(p->myCan_x, p->mbx_x, p->mbx_dir);
    myCan_MBX_ID_change (p->myCan_x, p->mbx_x, p->mbx_pkt.ID);
    myCan_MBX_DLC_change(p->myCan_x, p->mbx_x, p->mbx_pkt.Length);
    myCan_MBX_Enable    (p->myCan_x, p->mbx_x);
}

inline void myCan_P_set_IT(struct CAN_P *p, enum CAN_IT_CHANNEL_N N){
    myCan_MBX_IT_ON( p->myCan_x, p->mbx_x, N);
}
//////////////////////////////////////////////////////////


struct CAN_P  RX_Example  = {"接收Example", &myCana, MBX_0, MBX_DIR_RX, {0x0A700101, MBX_DLC_1, {0}}, 0};
struct CAN_P  TX_Example  = {"发送Example", &myCana, MBX_1, MBX_DIR_TX, {0x0A106115, MBX_DLC_3, {0}}, 0};

//根据 协议P , 配置 CAN 邮箱 MBX
void myCan_P_Config()
{
    myCan_P_set(& RX_Example );
    myCan_P_set(& TX_Example );
}

// 接收 使用中断的方式接收
void myCan_P_Config_IT()
{
    myCan_P_set_IT(& RX_Example , CAN_IT_CHANNEL_1); // CHANNEL_1: CAN-A 对应 Int_9.6  CAN-B 对应 Int_9.8

    /****************************************************
     *  注册中断服务函数
     *  开启中断组, 启用 Int_9.6
     ****************************************************/
    EALLOW;
    PieVectTable.ECAN1INTA = &Ecana_isr1;   //注册中断服务函数
    EDIS;
    PieCtrlRegs.PIEIER9.bit.INTx6 = 1;      //启用 Int 9.6
    IER |= M_INT9;                          //开启中断组
}

// Int 9.6   CAN 中断服务函数
__interrupt void Ecana_isr1(void)
{
    /****************************************************
     *   CAN-A 的中断1 即 IF1
     ****************************************************/
    if(ECanaRegs.CANGIF1.bit.GMIF1 == 1)
    {
        /****************************************************
         *   邮箱0 （即RMP0或RFP0） 的接收中断配置
         ****************************************************/
        //接收到“数据帧”
        if(ECanaRegs.CANRMP.bit.RMP0 == 1)
        {
            // your interrupt code here
            // ...

            // 清除标记位
            ECanaRegs.CANRMP.bit.RMP0 = 1;
        }
        //接收到“远程帧”
        else if(ECanaRegs.CANRFP.bit.RFP0 == 1)
        {
            // your interrupt code here
            // ...

            // 清除标记位
            ECanaRegs.CANRFP.bit.RFP0 = 1;
        }
    }
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}