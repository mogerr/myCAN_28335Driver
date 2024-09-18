#include "myCan_28335Driver.h"

/*****************************************************/
//  ECAN的寄存器不能单独配置单个bit, 必须一次性配置32个bit。
//  可用shadow的方式:
//  ECanaShadow.CANTIOC.all         = ECanaRegs.CANTIOC.all;    // Step 1
//  ECanaShadow.CANTIOC.bit.TXFUNC  = 1;                        // Step 2
//  ECanaRegs.CANTIOC.all           = ECanaShadow.CANTIOC.all;  // Step 3
/*****************************************************/

#include <string.h>


/** 
 * @brief 设置波特率
 * @param u 指向一个实例
 * @param bps 要设置的波特率
 */
inline void myCan_bps(struct myCan *u, enum CANBPS bps){
    struct ECAN_REGS ECanaShadow;
    EALLOW;
    /* Configure bit timing parameters for eCANA*/
	ECanaShadow.CANMC.all = u->pECanRegs->CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
    u->pECanRegs->CANMC.all = ECanaShadow.CANMC.all;

    ECanaShadow.CANES.all = u->pECanRegs->CANES.all;
    do{
	    ECanaShadow.CANES.all = u->pECanRegs->CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 1 );  		// Wait for CCE bit to be set..
    
    ECanaShadow.CANBTC.all = 0;
    
#if (CPU_FRQ_150MHZ)  // CPU_FRQ_150MHz is defined in DSP2833x_Examples.h
    /* The following block for all 150 MHz SYSCLKOUT (75 MHz CAN clock)
        See Note at End of File */
    //4：default. Bit rate = 1 Mbps
    ECanaShadow.CANBTC.bit.BRPREG = 4; u->bps = bps_1000;
    if(bps == bps_1000)    { ECanaShadow.CANBTC.bit.BRPREG = 4;  u->bps = bps_1000;}
    else if(bps == bps_500){ ECanaShadow.CANBTC.bit.BRPREG = 9;  u->bps = bps_500;}
    else if(bps == bps_250){ ECanaShadow.CANBTC.bit.BRPREG = 19; u->bps = bps_250;}
    ECanaShadow.CANBTC.bit.TSEG2REG = 2;
    ECanaShadow.CANBTC.bit.TSEG1REG = 10;
#endif
#if (CPU_FRQ_100MHZ) // CPU_FRQ_100MHz is defined in DSP2833x_Examples.h
	/* The following block is only for 100 MHz SYSCLKOUT (50 MHz CAN clock). Bit rate = 1 Mbps
	   See Note at End of File */
    //4：default. Bit rate = 1 Mbps
    ECanaShadow.CANBTC.bit.BRPREG = 4;
    if(bps == bps_1000)    { ECanaShadow.CANBTC.bit.BRPREG = 4; }
    else if(bps == bps_500){ ECanaShadow.CANBTC.bit.BRPREG = 9; }
    else if(bps == bps_250){ ECanaShadow.CANBTC.bit.BRPREG = 19; }
    ECanaShadow.CANBTC.bit.TSEG2REG = 1;
    ECanaShadow.CANBTC.bit.TSEG1REG = 6;
#endif
    
    ECanaShadow.CANBTC.bit.SAM = 1;
    u->pECanRegs->CANBTC.all = ECanaShadow.CANBTC.all;

    ECanaShadow.CANMC.all = u->pECanRegs->CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
    u->pECanRegs->CANMC.all = ECanaShadow.CANMC.all;

    ECanaShadow.CANES.all = u->pECanRegs->CANES.all;
    do{
       ECanaShadow.CANES.all = u->pECanRegs->CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 0 ); 

    EDIS;
}



/**
 * @brief 关闭邮箱
 * @param u 指向一个实例
 * @param MBX_num 邮箱编号
 */
void myCan_MBX_Disable(struct myCan *u, Uint32 MBX_num){
    union CANME_REG    CANME_Shadow; 
    union CANME_REG    CANME_Shadow_1; 
    volatile union CANME_REG *ME = &u->pECanRegs->CANME; 
    
    // Disable MBX
    CANME_Shadow.all  = ME->all;
    CANME_Shadow.all &= ~(1 << MBX_num);
    ME->all = CANME_Shadow.all;

    do{
        CANME_Shadow_1.all = ME->all;
    }while(CANME_Shadow_1.all != CANME_Shadow.all);
}


/**
 * @brief 开启邮箱
 * @param u 指向一个实例
 * @param MBX_num 邮箱编号
 */
void myCan_MBX_Enable(struct myCan *u, Uint32 MBX_num){
    union CANME_REG    CANME_Shadow; 
    union CANME_REG    CANME_Shadow_1; 
    volatile union CANME_REG *ME = &u->pECanRegs->CANME; 
    
    // Enable MBX
    CANME_Shadow.all  = ME->all;
    CANME_Shadow.all |= (1 << MBX_num);
    ME->all = CANME_Shadow.all;

    do{
        CANME_Shadow_1.all = ME->all;
    }while(CANME_Shadow_1.all != CANME_Shadow.all);
}



/**
 * @brief 改变 MBX 邮箱传输的方向。注意，改变方向前，先执行 失能MBX。
 * @param u 指向一个myCan实例
 * @param MBX_num 第几号MBX，范围0-15 （16 - 31 邮箱使用会出 bug）
 * @param DIR 0: MBX_DIR_TX   1: MBX_DIR_RX
 */
void myCan_MBX_DIR_change(struct myCan *u, enum MBX_N MBX_num, enum MBX_DIR  DIR){
    Uint32 ME_Shodaw;
    Uint32 MD_Shadow_1;
    Uint32 MD_Shadow_2;
    Uint32 Flag;
    Uint16  DIR_Temp;
    volatile union CANLAM_REG *LAM = (&u->pECanLAMRegs->LAM0 + MBX_num);
    volatile union CANMD_REG *MD = &u->pECanRegs->CANMD;
    volatile union CANME_REG *ME = &u->pECanRegs->CANME;
    // 范围 0-31
    MBX_num &= 0x1F;
    // 范围 0-1
    if(DIR == MBX_DIR_RX){
        DIR_Temp = 0x1;
    }else{
        DIR_Temp = 0x0;
    }

    // 有保护，首先需要关闭邮箱
    ME_Shodaw = ME->all;
    Flag = (ME->all) & (1 << MBX_num);
    // 如果邮箱未关闭，则关闭邮箱
    if(Flag != 0){
        ME_Shodaw = ME->all;
        ME_Shodaw &= ~(1 << MBX_num);
        ME->all = ME_Shodaw;
        do{
            Flag = (ME->all) & (1 << MBX_num);
        }while(Flag != 0);//直到邮箱关闭
        Flag = 1;
    }
    // This register can be written only when mailbox n is disable.
    // 只能用 .bit 的方式来改变 DIR ，用 .all 的方式有 bug 。
    // 后面发现 .bit .all 方式都有 bug ，没办法了，
    // 16 - 31 邮箱改变方向时会出问题，会全乱套，
    // 不知道什么原因导致的，应该和CAN电路有关
    // 解决不掉。所以 16 - 31 邮箱都不要使用了
    MD_Shadow_1 = MD->all;
    MD_Shadow_1 &= ~(1 << MBX_num);
    MD_Shadow_1 |= (DIR_Temp << MBX_num);
    if(MBX_num == 0)      { MD->bit.MD0 = DIR_Temp; }
    else if(MBX_num == 1) { MD->bit.MD1 = DIR_Temp; }
    else if(MBX_num == 2) { MD->bit.MD2 = DIR_Temp; }
    else if(MBX_num == 3) { MD->bit.MD3 = DIR_Temp; }
    else if(MBX_num == 4) { MD->bit.MD4 = DIR_Temp; }
    else if(MBX_num == 5) { MD->bit.MD5 = DIR_Temp; }
    else if(MBX_num == 6) { MD->bit.MD6 = DIR_Temp; }
    else if(MBX_num == 7) { MD->bit.MD7 = DIR_Temp; }
    else if(MBX_num == 8) { MD->bit.MD8 = DIR_Temp; }
    else if(MBX_num == 9) { MD->bit.MD9 = DIR_Temp; }
    else if(MBX_num == 10){ MD->bit.MD10 = DIR_Temp; }
    else if(MBX_num == 11){ MD->bit.MD11 = DIR_Temp; }
    else if(MBX_num == 12){ MD->bit.MD12 = DIR_Temp; }
    else if(MBX_num == 13){ MD->bit.MD13 = DIR_Temp; }
    else if(MBX_num == 14){ MD->bit.MD14 = DIR_Temp; }
    else if(MBX_num == 15){ MD->bit.MD15 = DIR_Temp; }
    else if(MBX_num == 16){ MD->bit.MD16 = DIR_Temp; }
    else if(MBX_num == 17){ MD->bit.MD17 = DIR_Temp; }
    else if(MBX_num == 18){ MD->bit.MD18 = DIR_Temp; }
    else if(MBX_num == 19){ MD->bit.MD19 = DIR_Temp; }
    else if(MBX_num == 20){ MD->bit.MD20 = DIR_Temp; }
    else if(MBX_num == 21){ MD->bit.MD21 = DIR_Temp; }
    else if(MBX_num == 22){ MD->bit.MD22 = DIR_Temp; }
    else if(MBX_num == 23){ MD->bit.MD23 = DIR_Temp; }
    else if(MBX_num == 24){ MD->bit.MD24 = DIR_Temp; }
    else if(MBX_num == 25){ MD->bit.MD25 = DIR_Temp; }
    else if(MBX_num == 26){ MD->bit.MD26 = DIR_Temp; }
    else if(MBX_num == 27){ MD->bit.MD27 = DIR_Temp; }
    else if(MBX_num == 28){ MD->bit.MD28 = DIR_Temp; }
    else if(MBX_num == 29){ MD->bit.MD29 = DIR_Temp; }
    else if(MBX_num == 30){ MD->bit.MD30 = DIR_Temp; }
    else if(MBX_num == 31){ MD->bit.MD31 = DIR_Temp; }
    do {
        MD_Shadow_2 = (MD->all) ;
    } while(MD_Shadow_2 != MD_Shadow_1);

    if(DIR == MBX_DIR_RX){
        /*
          LAM[28:0]
          这些位启用一个进入消息的任意标识符位的屏蔽。
          1 针对接受到的标识符的相应位， 接受一个 0 或 1（ 无关） 。
          0 接收到的标识符位值必须与 MSGID 寄存器的相应标识符位相匹配。
        */
        LAM->all = 0x0000000;
        /*
          LAMI 本地接受屏蔽标识符扩展位
          1 可以接收标准和扩展帧。 在扩展帧的情况下， 标识符的所有 29 位被存储在邮箱中， 本地接受屏蔽寄
          存器的所有 29 位被过滤器使用。 在一个标准帧的情况下， 只有标识符的头 11 个位（ 28 至 18 位）
          和本地接受屏蔽被使用。
          0 存储在邮箱中的标识符扩展位决定了哪些消息应该被接收到
        */
        LAM->bit.LAMI = 1;
    }

    // 还原 ME 状态，判断是否需要重启邮箱
    if(Flag != 0){
        ME_Shodaw = ME->all;
        ME_Shodaw |= (1<<MBX_num);
        ME->all = ME_Shodaw;
        do{
            Flag = (ME->all) & (1 << MBX_num);
        }while(Flag == 0);//直到邮箱开启
    }
}


/**
 * @brief 改变MBX中的寄存器中的MSGID字段，改变帧ID，使用扩展帧(extended ID)
 *         注意，改变ID前，先进行 失能MBX
 * @param u 指向一个myCan实例
 * @param MBX_num 第几号MBX，范围0-31
 * @param ID 帧ID的目标值，29bit, 28:0 这个是由MBX寄存器硬件来决定的
 */
void myCan_MBX_ID_change(struct myCan *u, enum MBX_N MBX_num, Uint32 ID){
    volatile struct MBOX *Mailbox;
    union CANMSGID_REG MSGIDShadow;
    // 范围是 0-31
    MBX_num &= 0x1F; 
    // if(MBX_num <=31 && MBX_num >=16){
    //     Mailbox = &u->pMBX->MBOX16 + MBX_num - 16;
    // }
    // else if(MBX_num <=16 && MBX_num >= 0){
    //     Mailbox = &u->pMBX->MBOX0 + MBX_num;
    // }
    Mailbox = &u->pMBX->MBOX0 + MBX_num;
    // MSG ID can only be used in 29 bits, which is 28:0
    ID &= (0x1FFFFFFF);
    // use an extended identifier (29 bits)
    MSGIDShadow.all = Mailbox->MSGID.all;
    MSGIDShadow.all &= ~(0x1FFFFFFF);
    MSGIDShadow.all |= ID;
    // use an extended identifier (29 bits), use this bit.
    MSGIDShadow.bit.IDE = 1;
    Mailbox->MSGID.all = MSGIDShadow.all;
}


/**
 * @brief 改变MBX的 DATA_LENGTH ，使邮箱发送/接收指定的数据长度，范围是0~8，单位是字节
 * @param u 指向一个myCan实例
 * @param MBX_num 第几个邮箱 range 0-31
 * @param Len 目标值, range 0~8，单位字节.
 */
void myCan_MBX_DLC_change(struct myCan *u, enum MBX_N MBX_num, Uint8 Len){
    volatile struct MBOX *Mailbox;
    Mailbox = &u->pMBX->MBOX0 + MBX_num; 
    // DLC (Data-length code) reg is 3:0, and valid value range is from 0 to 8
    Len = Len & 0x0F;
    if(Len > 8) {Len = 8;}
    Mailbox->MSGCTRL.bit.DLC = Len;
}



/**
 * @brief 改变 Timout 设置， 每个邮箱都有单独的超时设置
 * @param u 指向一个myCan实例
 * @param MBX_num 第几个邮箱 range 0-31
 * @param TO TimeOut Setting Value. 32bits. 
 *  The reg CANTSC is a free-running 32-bit timer which is clocked by the bit clock of the CAN bus. 
 *  For example, at a bit rate of 1 Mbps, CANTSC would increment every 1 μs. 
 *  When TO set to 0, it is set to no timeout. 如果设置为TO=0，那么表示不设置超时。
 */
//void myCan_MBX_TO_change(struct myCan *u, enum MBX_N MBX_num, Uint32 TO){
//    // volatile Uint32 *pMOTO = &(u->pECanMOTORegs->MOTO0) + MBX_num;
//    // *pMOTO = TO;
//    *(&u->TO.MOTO0 + MBX_num) = TO;
//}


/**
 * @brief This func reads the indicated mailbox data, include ID, MDL, MDH, Length
 *        识别到接收数据后执行这条函数，可以读取MBX寄存器中的值，包括接收数据的长度信息
 * @param u 指向一个实例
 * @param MBX_num range 0-31
 * @param RX_PKT 指针，指向一个缓存 Packet（PKT）
 */
void myCan_MBX_MD2pkt(struct myCan *u, enum MBX_N MBX_num, struct myCan_PKT *RX_PKT){
   volatile struct MBOX *Mailbox;
   Mailbox = &u->pMBX->MBOX0 + MBX_num;
   RX_PKT->Length = Mailbox->MSGCTRL.bit.DLC;      // Can 的 Data-Length-Code（接收到的数据的长度）
   RX_PKT->ID     = Mailbox->MSGID.all & 0x1FFFFFFF;   // Can 的帧ID (29bit)
   RX_PKT->MD[0] = Mailbox->MDL.byte.BYTE0;    // MDL 32位
   RX_PKT->MD[1] = Mailbox->MDL.byte.BYTE1;
   RX_PKT->MD[2] = Mailbox->MDL.byte.BYTE2;
   RX_PKT->MD[3] = Mailbox->MDL.byte.BYTE3;
   RX_PKT->MD[4] = Mailbox->MDH.byte.BYTE4;    // MDH 32位
   RX_PKT->MD[5] = Mailbox->MDH.byte.BYTE5;
   RX_PKT->MD[6] = Mailbox->MDH.byte.BYTE6;
   RX_PKT->MD[7] = Mailbox->MDH.byte.BYTE7;
}


/**
 * @brief 将PKT中的MD数据写入MD寄存器，等待发送
 *          耗时 718ns_@_RUN_From_RAM_@_CANCLK_75MHz_SYSOUTCLK_150MHz
 * @param u 指向一个myCan实例
 * @param MBX_num range 0-31
 * @param TX_PKT 指向一个缓存 Packet（PKT）
 */
void myCan_MBX_pkt2MD(struct myCan *u, enum MBX_N MBX_num, struct myCan_PKT *TX_PKT){
    struct MBOX MBXShadow;
    volatile struct MBOX *Mailbox; 
    Mailbox = &u->pMBX->MBOX0 + MBX_num;
    MBXShadow.MDL.all = 0x00000000;
    MBXShadow.MDL.byte.BYTE0 = TX_PKT->MD[0];
    MBXShadow.MDL.byte.BYTE1 = TX_PKT->MD[1];
    MBXShadow.MDL.byte.BYTE2 = TX_PKT->MD[2];
    MBXShadow.MDL.byte.BYTE3 = TX_PKT->MD[3];
    MBXShadow.MDH.all = 0x00000000;
    MBXShadow.MDH.byte.BYTE4 = TX_PKT->MD[4];
    MBXShadow.MDH.byte.BYTE5 = TX_PKT->MD[5];
    MBXShadow.MDH.byte.BYTE6 = TX_PKT->MD[6];
    MBXShadow.MDH.byte.BYTE7 = TX_PKT->MD[7];
    Mailbox->MDL.all = MBXShadow.MDL.all;
    Mailbox->MDH.all = MBXShadow.MDH.all;
}


/**
 * @brief 开始传输MD寄存器，（前提是MD已被更新）
 *        耗时 630ns_@_RUN_From_RAM_@_CANCLK_75MHz_SYSOUTCLK_150MHz
 * @param u 指向一个myCan实例
 * @param MBX_num range 0-31
 */
void myCan_MBX_Trans(struct myCan *u, enum MBX_N MBX_num){
    union CANTRS_REG TRSShadow;
    union CANTOC_REG TOC_Shadow;

    //关闭硬件超时
    TOC_Shadow.all = u->pECanRegs->CANTOC.all;
    TOC_Shadow.all &= ~(1 << MBX_num);
    u->pECanRegs->CANTOC.all = TOC_Shadow.all;
    *(&(u->pECanMOTORegs->MOTO0) + MBX_num) = 0x0;

    // 进行发送请求
    // DATASHEET: These bits are normally set by the CPU and cleared by the CAN module logic
    TRSShadow.all = u->pECanRegs->CANTRS.all;
    TRSShadow.all |= 1 << MBX_num;             // Set TRS for mailbox transport, range 0~31
    u->pECanRegs->CANTRS.all = TRSShadow.all;
}


/**
 * @brief 阻塞的方式传输MD寄存器，（前提是MD已被更新）
 *        使用此函数时，CAN物理连线一定要连上，不然会卡在while中.
 *        实测 250_kbps 的情况下，等待 363us 发送完成
 * @param u 指向一个myCan实例
 * @param MBX_num range 0-31
 */
void myCan_MBX_Trans_withBlock(struct myCan *u, enum MBX_N MBX_num){
    union CANTRS_REG TRSShadow;
    union CANTOC_REG TOC_Shadow;
    Uint32 TAShadow;

    //关闭硬件超时
    TOC_Shadow.all = u->pECanRegs->CANTOC.all;
    TOC_Shadow.all &= ~(1 << MBX_num);
    u->pECanRegs->CANTOC.all = TOC_Shadow.all;
    *(&(u->pECanMOTORegs->MOTO0) + MBX_num) = 0x0;

    //等待上一个数据发送完成
    do{
        // DATASHEET: These bits are normally set by the CPU and cleared by the CAN module logic
        TRSShadow.all = u->pECanRegs->CANTRS.all;
        TRSShadow.all &= (1 << MBX_num);
    }while(TRSShadow.all != 0);

    // 进行发送请求
    // DATASHEET: These bits are normally set by the CPU and cleared by the CAN module logic
    TRSShadow.all = u->pECanRegs->CANTRS.all;
    TRSShadow.all |= (1 << MBX_num);             // Set TRS for mailbox transport, range 0~31
    u->pECanRegs->CANTRS.all = TRSShadow.all;

    //等待发送完成
    do{
        TAShadow = u->pECanRegs->CANTA.all;
        TAShadow &= (1 << MBX_num);
    }while(TAShadow == 0);
    //清除TA位
    u->pECanRegs->CANTA.all = TAShadow;
}


/**
 * @brief 开始传输MD寄存器，（前提是MD已被更新）
 *        耗时 848ns_@_RUN_From_RAM_@_CANCLK_75MHz_SYSOUTCLK_150MHz
 * @param u 指向一个myCan实例
 * @param MBX_num range 0-31
 * @param TimeOut 单位ms
 */
void myCan_MBX_Trans_withTimeOut(struct myCan *u, enum MBX_N MBX_num, Uint32 TO_ms){
    union CANTRS_REG TRSShadow;
    union CANTOC_REG TOC_Shadow;
    volatile Uint32 *pMOTO = &(u->pECanMOTORegs->MOTO0) + MBX_num;
    Uint32 MOTO_Temp;
    Uint32 TimeOut_Temp;

    // 使能or失能 超时硬件控制
    if(TO_ms == 0x0){
        TOC_Shadow.all = u->pECanRegs->CANTOC.all;
        TOC_Shadow.all &= ~(1 << MBX_num);
        u->pECanRegs->CANTOC.all = TOC_Shadow.all;
        *pMOTO = 0x0;
    }
    else{
        TimeOut_Temp = TO_ms * u->bps;
        MOTO_Temp = u->pECanRegs->CANTSC;
        MOTO_Temp += TimeOut_Temp;
        *pMOTO = MOTO_Temp;
        TOC_Shadow.all = u->pECanRegs->CANTOC.all;
        TOC_Shadow.all |=  (1 << MBX_num);
        u->pECanRegs->CANTOC.all = TOC_Shadow.all;
    }

    // 进行发送请求
    // DATASHEET: These bits are normally set by the CPU and cleared by the CAN module logic
    TRSShadow.all = u->pECanRegs->CANTRS.all;
    TRSShadow.all |= (1 << MBX_num);             // Set TRS for mailbox transport, range 0~31
    u->pECanRegs->CANTRS.all = TRSShadow.all;
}


/****************************************************
 *   邮箱 MBX_num 开启中断
 *   接收方式推荐使用中断接收
 ****************************************************/
void myCan_MBX_IT_ON(struct myCan *u, enum MBX_N MBX_num, enum CAN_IT_CHANNEL_N N){
    Uint32 Shadow;
    Shadow  = u->pECanRegs->CANMIM.all;
    Shadow |= (1<<MBX_num);
    EALLOW;
    u->pECanRegs->CANMIM.all = Shadow;     //使能中断，邮箱 MBX_num 的中断;
    EDIS;
    /*
     * CAN 有 中断1 和 中断0 可选
     * MIL=0 -> Int 9.5   MIL=1 -> Int 9.6
     */
    Shadow = u->pECanRegs->CANMIL.all;
    Shadow &= ~(1 << MBX_num);
    Shadow |=  (N << MBX_num);
    EALLOW;
    u->pECanRegs->CANMIL.all = Shadow;
    if(N == CAN_IT_CHANNEL_0)       u->pECanRegs->CANGIM.bit.I0EN = 1;//使能中断0;
    else if(N == CAN_IT_CHANNEL_1)  u->pECanRegs->CANGIM.bit.I1EN = 1;//使能中断1;
    EDIS;
}




inline void __MBX_CLR(struct myCan *u){
    volatile struct MBOX *MBX = &u->pMBX->MBOX0;
    Uint8 i = 0;
    for(i=0; i<32; i++){
        MBX->MSGID.all = 0x00000000;
        MBX->MSGCTRL.all = 0x00000000;
        MBX->MDL.all = 0x00000000;
        MBX->MDH.all = 0x00000000;
        MBX = MBX +1;
    }
}

inline void __MOTO_CLR(struct myCan *u){
    volatile Uint32 *MOTO_Shadow = &u->pECanMOTORegs->MOTO0;
    Uint8 i = 0;
    for(i=0; i<32; i++){
        *(MOTO_Shadow + i) = 0x00000000;
    }
}

inline void __MOTS_CLR(struct myCan *u){
    volatile Uint32 *MOTS_Shadow = &u->pECanMOTSRegs->MOTS0;
    Uint8 i = 0;
    for(i=0; i<32; i++){
        *(MOTS_Shadow + i) = 0x00000000;
    }
}

inline void __ME_CLR(struct myCan *u){
    volatile union CANME_REG *ME = &u->pECanRegs->CANME; 
    union CANME_REG ME_Shadow;
    // Disable All MBX
    ME->all = 0x00000000; 
    do{
        ME_Shadow.all = ME->all;
    }while(ME_Shadow.all != 0x00000000);
}

/* 可释放注释
// 作用是启用所有邮箱
inline void __ME_SET(struct myCan *u){
    volatile union CANME_REG *ME = &u->pECanRegs->CANME;
    union CANME_REG ME_Shadow;
    // Disable All MBX
    ME->all = 0xFFFFFFFF;
    do{
        ME_Shadow.all = ME->all;
    }while(ME_Shadow.all != 0xFFFFFFFF);
}
可释放注释 */


/**
 * @brief SCB=1 表示开启 eCAN 模式，从而开启32个邮箱，否则16个邮箱。
 *        同时关闭 STM 模式。
 * @param u 
 */
inline void __MBX_32(struct myCan *u){
    struct ECAN_REGS ECanaShadow;
    //
    EALLOW;

    // SCB
	ECanaShadow.CANMC.all = u->pECanRegs->CANMC.all;
	ECanaShadow.CANMC.bit.SCB = 1;
	u->pECanRegs->CANMC.all = ECanaShadow.CANMC.all;
    do{
        ECanaShadow.CANMC.all = u->pECanRegs->CANMC.all;
    }while(ECanaShadow.CANMC.bit.SCB != 1);

    // STM
    ECanaShadow.CANMC.all = u->pECanRegs->CANMC.all;
	ECanaShadow.CANMC.bit.STM = 0;
	u->pECanRegs->CANMC.all = ECanaShadow.CANMC.all;
    do{
        ECanaShadow.CANMC.all = u->pECanRegs->CANMC.all;
    }while(ECanaShadow.CANMC.bit.STM != 0);

    // 
    EDIS;
}


/**
 * @brief 轮询的方式，poll是否有“发送成功”事件
 * @param u 
 */
void __myCan_TransAck(struct myCan *u){
    Uint32 MBX_bit_i;
    Uint32 FLAG;
    enum MBX_N  i;
    // 发送成功 寄存器
    volatile union CANTA_REG   *TA  = &u->pECanRegs->CANTA;
    //volatile union CANTOC_REG  *TOC = &u->pECanRegs->CANTOC;

    //
    // 没有“发送成功”事件发生
    if(TA->all == 0x00000000){
        return;
    }
    //
    //************************************************/
    /*  以下留给用户，可修改相对应的 MBX_bit_i 发送成功事件  */
    /*************************************************/
    //
    //
    for(i=MBX_0; i<=MBX_31; i++){
        // 如果 MBX_i 发生发送成功事件
        MBX_bit_i = (1 << i);
        FLAG = TA->all;
        FLAG = (FLAG & MBX_bit_i);
        if(FLAG != 0){
            // CPU频率大于CAN寄存器频率，
            // 轮询时，用 do-while 确保 TOS 置0，确保用户操作不会被重复执行。
            do{
                // 清除 TA 相应位，开始下一次检测. 
                // resets the bits in CANTA by writing a 1. 
                TA->all = MBX_bit_i;
                //
                FLAG = (TA->all & MBX_bit_i);
            }while(FLAG != 0);
            //
            // 用户操作
            myCan_TransAck_Callback(u, i);
        }
    }
}


/**
 * @brief 轮询的方式，poll是否有“超时”事件
 * @param u 
 */
inline void __myCan_TimeOut(struct myCan *u){
    Uint32 MBX_bit_i;
    Uint32 FLAG;
    enum MBX_N  i;
    // 超时 寄存器
    volatile union CANTOS_REG   *TOS = &u->pECanRegs->CANTOS;
    volatile union CANTOC_REG   *TOC = &u->pECanRegs->CANTOC;
    // 发送请求 寄存器   //TRR: Trans Reset
    volatile union CANTRR_REG   *TRR = &u->pECanRegs->CANTRR;
    // 接收请求 寄存器
    

    //
    // 没有超时事件发生
    if(TOS->all == 0x00000000){
        return;
    }
    //
    /**********************************************/
    /*  以下留给用户，可修改相对应的 MBX_bit_i 超时事件  */
    /**********************************************/
    //
    ////////  如果 MBX_i 发生超时事件  /////////
    //
    for(i=MBX_0; i<=MBX_31; i++){
        MBX_bit_i = (1 << i);
        FLAG = (TOS->all & MBX_bit_i);
        if(FLAG != 0){
            // CPU频率大于CAN寄存器频率，
            // 轮询时，用 do-while 确保 TOS 置0，确保用户操作不会被重复执行。
            do{
                // 清除超时控制标记。方法是 TOC 置0, 那么 TOS 会被清除。
                // 清除 TOS ，关闭下一次超时。
                FLAG = TOC->all;
                FLAG &= ~(MBX_bit_i);
                TOC->all = FLAG;
                // 超时就不再 发送数据。
                // 方法是 TRR 置1 。 TRR：Trans Reset
                FLAG = TRR->all;
                FLAG |= (MBX_bit_i);
                TRR->all = FLAG;
                //清除MOTO
                *(&u->pECanMOTORegs->MOTO0 + i) = 0x0;
                //判断是否跳出While
                FLAG = (TOS->all & MBX_bit_i);
            }while( FLAG != 0);
            //
            // 用户操作
            myCan_TimeOut_Callback(u, i);
        }
    }
}



/* 可释放注释 */
// /**
//  * @brief 轮询的方式，poll是否有“接收到远程数据帧”的事件发生
//  * @param u
//  */
// inline void __myCan_RecvRMP(struct myCan *u){
//     Uint32 MBX_bit_i;
//     Uint32 FLAG;
//     enum MBX_N  i;
//     // RMP 寄存器
//     volatile union CANRMP_REG *RMP = &u->pECanRegs->CANRMP;
//     //
//     // 没有数据接收
//     if(RMP->all == 0x00000000){
//         return;
//     }
//     //
//     //////////////////////////////////////////////////
//     //  以下留给用户，可修改相对应的 MBX_bit_i 数据接收事件      //
//     //////////////////////////////////////////////////
//     //
//     /////////  如果 MBX_i 发接收到“数据帧”事件  ////////
//     //
//     for(i=MBX_0; i<=MBX_31; i++){
//         MBX_bit_i = (1 << i);
//         FLAG = (RMP->all & MBX_bit_i);
//         if( FLAG != 0){
//             // CPU频率大于CAN寄存器频率，
//             // 轮询时，用 do-while 确保 RMP 置0，确保用户操作不会被重复执行。
//             do{
//                 // 清除 RMP . The bit are reset by writing a 1 from the CPU.
//                 FLAG = RMP->all;
//                 FLAG |= MBX_bit_i;
//                 RMP->all = FLAG;
//                 //
//                 FLAG = (RMP->all & MBX_bit_i);
//             }while( FLAG != 0);
//             //
//             // 用户操作
//             myCan_RecvRMP_Callback(u, i);
//         }
//     }
// }
/* 可释放注释 */




/* 可释放注释 */
// /**
//  * @brief 轮询的方式，poll是否存在“接收到远程请求帧”的事件
//  * @param u
//  */
// inline void __myCan_RecvRFP(struct myCan *u){
//     Uint32 MBX_bit_i;
//     Uint32 FLAG;
//     enum MBX_N  i;
//     // RFP 寄存器
//     volatile union CANRFP_REG *RFP = &u->pECanRegs->CANRFP;
//     //
//     // 没有数据接收
//     if(RFP->all == 0x00000000){
//         return;
//     }
//     //
//     //////////////////////////////////////////////////
//     //  以下留给用户，可修改相对应的 MBX_bit_i 数据接收事件      //
//     //////////////////////////////////////////////////
//     //
//     ////////  如果 MBX_i 接收到“远程帧”事件  ////////
//     //
//     for(i=MBX_0; i<=MBX_31; i++){
//         MBX_bit_i = (1 << i);
//         FLAG = (RFP->all & MBX_bit_i);
//         if( FLAG != 0){
//             // CPU频率大于CAN寄存器频率，
//             // 轮询时，用 do-while 确保 RFP 置0，确保用户操作不会被重复执行。
//             do{
//                 // 清除 RFP . The bit are reset by writing a 1 from the CPU.
//                 FLAG = RFP->all;
//                 FLAG |= MBX_bit_i;
//                 RFP->all = FLAG;
//                 //
//                 FLAG = (RFP->all & MBX_bit_i);
//             }while( FLAG != 0);
//             //
//             // 用户操作
//             myCan_RecvRFP_Callback(u, i);
//         }
//     }
// }
/* 可释放注释 */




struct myCan myCana = {0};    // CANa 实例
#if (DSP28_ECANB)
struct myCan myCanb = {0};    // CANb 实例
#endif

/**
 * @brief 全局实例的初始化
 */
inline void myCan_VarInit(){
    // 启用eCan模式
    // 16-31 邮箱有bug，不要用
    __MBX_32(&myCana);

    // a
    myCana.pECanRegs = &ECanaRegs;
    myCana.pMBX      = &ECanaMboxes; 
    myCana.pECanMOTORegs = &ECanaMOTORegs;
    myCana.pECanMOTSRegs = &ECanaMOTSRegs;
    __MOTO_CLR(&myCana);
    __MOTS_CLR(&myCana);
    __MBX_CLR(&myCana);
    __ME_CLR(&myCana);

#if (DSP28_ECANB)
    // 启用eCan模式
    // 16-31 邮箱有bug，不要用
    __MBX_32(&myCanb);

    // b
    myCanb.pECanRegs = &ECanbRegs;
    myCanb.pMBX      = &ECanbMboxes; 
    myCanb.pECanMOTORegs = &ECanbMOTORegs;
    myCanb.pECanMOTSRegs = &ECanbMOTSRegs;
    __MOTO_CLR(&myCanb);
    __MOTS_CLR(&myCanb);
    __MBX_CLR(&myCanb);
    __ME_CLR(&myCanb);
#endif
}




/**
 * @brief 初始化
 */
void myCan_Init(){

    // 实例初始化  
    myCan_VarInit();

    // GPIO初始化
    InitECanGpio();

    // 外设设备初始化
    InitECan();

    // 修改 bps
    myCan_bps(&myCana, bps_250);

#if (DSP28_ECANB)
    myCan_bps(&myCanb, bps_250);
#endif

    // 16-31 邮箱有bug，不要用
    

}






/**
 * @brief myCan Test Function Poll
 */
inline void myCan_TestPoll(struct myCan *u, enum MBX_N MBX_num){
    struct myCan_PKT PKT = {0};
    static Uint32  TO = 0x30000;  // 1.3s@0x50000  786ms@0x30000  262ms@0x10000
    static Uint32  TSC_Shadow = 0x0;
    
    //未到定时，退出
    if(ECanaRegs.CANTSC - TSC_Shadow < TO) {
        return;
    }
    TSC_Shadow = ECanaRegs.CANTSC;

    *(PKT.MD+0) = 0x56;
    *(PKT.MD+1) = 0x78;
    *(PKT.MD+2) = 0x9A;
    *(PKT.MD+3) = 0xBC;
    *(PKT.MD+4) = 0xDE;
    *(PKT.MD+5) = 0xF0;
    *(PKT.MD+6) = 0x12;
    *(PKT.MD+7) = 0x34;

    myCan_MBX_pkt2MD(u, MBX_num, &PKT);

    myCan_MBX_Trans_withTimeOut(u, MBX_num, 200);
//    myCan_MBX_Trans(u, MBX_num);
//    myCan_MBX_Trans_withBlock(u, MBX_num);

}




/**
 * @brief 轮询（poll）函数，放在 main 的主循环之中运行
 * @TimerInterval  Poll的周期，比如50ms定时Poll一次，那么TimeInterval=50000
 */
void myCan_poll(){


    // 测试用
    // myCan_TestPoll(&myCana, MBX_1);

    // 检测到 MBX 存在“硬件超时”事件
    __myCan_TimeOut(&myCana);

    // 检测到 MBX 存在“发送成功”事件
    //  __myCan_TransAck(&myCana);


/* 已使用中断的方式处理CAN接收，这部分注释掉
 *
 *   // 检测到 MBX 存在"数据帧"接收
 *   // __myCan_RecvRMP(&myCana);
 *
 *   // 检测到 MBX 存在"远程帧"接收
 *   //  __myCan_RecvRFP(&myCana);
 *
 */

}







/**
 * @brief “传输超时” 事件发生时，会被执行一次
 * @param u 回调，传入一个指针
 * @param MBX_num 回调，传入一个邮箱编号
 */
void myCan_TimeOut_Callback(struct myCan *u, enum MBX_N MBX_num){
//    asm ("      ESTOP0");
    return;
}


/**
 * @brief  “传输完成” 事件发生时，会被执行一次
 * @param u 回调，传入一个指针
 * @param MBX_num 回调，传入一个邮箱编号
 */
void myCan_TransAck_Callback(struct myCan *u, enum MBX_N MBX_num){
//    asm ("      ESTOP0");
    return;
}


/**
 * @brief  接收到“数据帧”的时候，会被执行一次 (注意：远程帧 ≠ 数据帧)
 * Received-Message-Pending Register (RMP)
 * @param u 回调，传入一个指针
 * @param MBX_num 回调，传入一个邮箱编号
 */
void myCan_RecvRMP_Callback(struct myCan *u, enum MBX_N MBX_num){
//    asm ("      ESTOP0");
    return;
}


/**
 * @brief  接收到“远程帧”的时候，会被执行一次 (注意：远程帧 ≠ 数据帧)
 * Remote-Frame-Pending Register (RFP)
 * @param u 回调，传入一个指针
 * @param MBX_num 回调，传入一个邮箱编号
 */
void myCan_RecvRFP_Callback(struct myCan *u, enum MBX_N MBX_num){
//    asm ("      ESTOP0");
    return;
}









