#ifndef _MYCAN_28225DRIVER_H_
#define _MYCAN_28225DRIVER_H_

typedef unsigned char Uint8;

// 设置邮箱传输方向
enum MBX_DIR{
    MBX_DIR_TX = 0,
    MBX_DIR_RX = 1
};
//#define MBX_DIR_TX 0
//#define MBX_DIR_RX 1

// 设置 数据长度控制（DLC）
enum MBX_DLC_X{
     MBX_DLC_0 = 0x0,
     MBX_DLC_1 = 0x1,
     MBX_DLC_2 = 0x2,
     MBX_DLC_3 = 0x3,
     MBX_DLC_4 = 0x4,
     MBX_DLC_5 = 0x5,
     MBX_DLC_6 = 0x6,
     MBX_DLC_7 = 0x7,
     MBX_DLC_8 = 0x8
};

//设置邮箱编号
enum MBX_N{
    MBX_0  = 0,
    MBX_1  = 1,
    MBX_2  = 2,
    MBX_3  = 3,
    MBX_4  = 4,
    MBX_5  = 5,
    MBX_6  = 6,
    MBX_7  = 7,
    MBX_8  = 8,
    MBX_9  = 9,
    MBX_10 = 10,
    MBX_11 = 11,
    MBX_12 = 12,
    MBX_13 = 13,
    MBX_14 = 14,
    MBX_15 = 15,
    MBX_16 = 16,
    MBX_17 = 17,
    MBX_18 = 18,
    MBX_19 = 19,
    MBX_20 = 20,
    MBX_21 = 21,
    MBX_22 = 22,
    MBX_23 = 23,
    MBX_24 = 24,
    MBX_25 = 25,
    MBX_26 = 26,
    MBX_27 = 27,
    MBX_28 = 28,
    MBX_29 = 29,
    MBX_30 = 30,
    MBX_31 = 31
};

// can bps 
enum CANBPS{
    bps_1000 = 1000,
    bps_500  = 500,
    bps_250  = 250
};

enum CAN_IT_CHANNEL_N{
    CAN_IT_CHANNEL_0 = 0,  //CAN-A 对应 Int_9.5   //CAN-B 对应 Int_9.7
    CAN_IT_CHANNEL_1 = 1   //CAN-A 对应 Int_9.6   //CAN-B 对应 Int_9.8
};

// 一个数据包 Packet（PKT），用于读取数据
struct myCan_PKT
{
    Uint32  ID:29;              //目前只写了 扩展ID 没有写 标准ID
    Uint8   Length:4;
    Uint8   MD[8];
};

// CAN Driver 的结构体
struct myCan
{
    // 寄存器管理
    volatile struct ECAN_REGS *pECanRegs;
    volatile struct ECAN_MBOXES *pMBX;
    volatile struct MOTO_REGS *pECanMOTORegs;
    volatile struct MOTS_REGS *pECanMOTSRegs;
    volatile struct LAM_REGS *pECanLAMRegs;
    //
    enum CANBPS bps;
};








extern struct myCan myCana;    // CANa 实例
#if (DSP28_ECANB)
extern struct myCan myCanb;    // CANb 实例
#endif

#pragma CODE_SECTION(myCan_poll, "ramfuncs");

void    myCan_Init();
void    myCan_poll();

void myCan_MBX_Disable   (struct myCan *u, Uint32 MBX_num);
void myCan_MBX_Enable    (struct myCan *u, Uint32 MBX_num);
void myCan_MBX_DIR_change(struct myCan *u, enum MBX_N MBX_num, enum MBX_DIR  DIR);
void myCan_MBX_ID_change (struct myCan *u, enum MBX_N MBX_num, Uint32 ID);
void myCan_MBX_DLC_change(struct myCan *u, enum MBX_N MBX_num, Uint8 Len);
void myCan_MBX_IT_ON     (struct myCan *u, enum MBX_N MBX_num, enum CAN_IT_CHANNEL_N N);


#pragma CODE_SECTION(myCan_MBX_MD2pkt, "ramfuncs");
#pragma CODE_SECTION(myCan_MBX_pkt2MD, "ramfuncs");
#pragma CODE_SECTION(myCan_MBX_Trans,  "ramfuncs");
#pragma CODE_SECTION(myCan_MBX_Trans_withBlock,   "ramfuncs");
#pragma CODE_SECTION(myCan_MBX_Trans_withTimeOut, "ramfuncs");

void myCan_MBX_MD2pkt(struct myCan *u, enum MBX_N MBX_num, struct myCan_PKT *RX_PKT);
void myCan_MBX_pkt2MD(struct myCan *u, enum MBX_N MBX_num, struct myCan_PKT *TX_PKT);
void myCan_MBX_Trans (struct myCan *u, enum MBX_N MBX_num);
void myCan_MBX_Trans_withBlock  (struct myCan *u, enum MBX_N MBX_num);
void myCan_MBX_Trans_withTimeOut(struct myCan *u, enum MBX_N MBX_num, Uint32 TimeOut);


void    myCan_TimeOut_Callback (struct myCan *u, enum MBX_N MBX_num);
void    myCan_TransAck_Callback(struct myCan *u, enum MBX_N MBX_num);
void    myCan_RecvRMP_Callback (struct myCan *u, enum MBX_N MBX_num);
void    myCan_RecvRFP_Callback (struct myCan *u, enum MBX_N MBX_num);


#endif
