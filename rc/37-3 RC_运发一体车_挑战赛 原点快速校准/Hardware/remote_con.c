#include "remote_con.h"
Remote_value RC_Ctrl;
void RemoteDataProcess(uint8_t *pData)
{
    RC_Ctrl.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    RC_Ctrl.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_Ctrl.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_Ctrl.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF;

    RC_Ctrl.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_Ctrl.s2 = ((pData[5] >> 4) & 0x0003);
    // your control code
    RC_Ctrl.ch0 = (RC_Ctrl.ch0 - 1024);
    RC_Ctrl.ch1 = (RC_Ctrl.ch1 - 1024);
    RC_Ctrl.ch2 = (RC_Ctrl.ch2 - 1024);
    RC_Ctrl.ch3 = (RC_Ctrl.ch3 - 1024);

    if (RC_Ctrl.ch0 > -10 && RC_Ctrl.ch0 < 10)
    {
        RC_Ctrl.ch0 = 0;
    }
    if (RC_Ctrl.ch1 > -10 && RC_Ctrl.ch1 < 10)
    {
        RC_Ctrl.ch1 = 0;
    }
    if (RC_Ctrl.ch2 > -10 && RC_Ctrl.ch2 < 10)
    {
        RC_Ctrl.ch2 = 0;
    }
    if (RC_Ctrl.ch3 > -10 && RC_Ctrl.ch3 < 10)
    {
        RC_Ctrl.ch3 = 0;
    }
}

int8_t Channel_optimization(int16_t value)
{
    int8_t a;
    switch (value)
    {
    case 191:
        a = 1;
        break;
    case 1792:
        a = 3;
        break;

    case 172:
        a = 1;
        break;
    case 992:
        a = 2;
        break;
    case 997:
        a = 2;
        break;
    case 1810:
        a = 3;
        break;
    }
    return a;
}
ELRS Elrs;
void ELRSDataProcess(uint8_t *rdata)
{
    // 10个通道的数据处理
    Elrs.ch[1] = ((rdata[3] >> 0) | (rdata[4] << 8)) & 0x07FF;
    Elrs.ch[2] = ((rdata[4] >> 3) | (rdata[5] << 5)) & 0x07FF;
    Elrs.ch[3] = ((rdata[5] >> 6) | (rdata[6] << 2) | (rdata[7] << 10)) & 0x07FF;
    Elrs.ch[4] = ((rdata[7] >> 1) | (rdata[8] << 7)) & 0x07FF;
    Elrs.ch[5] = ((rdata[8] >> 4) | (rdata[9] << 4)) & 0x07FF;
    Elrs.ch[6] = ((rdata[9] >> 7) | (rdata[10] << 1) | (rdata[11] << 9)) & 0x07FF;
    Elrs.ch[7] = ((rdata[11] >> 2) | (rdata[12] << 6)) & 0x07FF;
    Elrs.ch[8] = ((rdata[12] >> 5) | (rdata[13] << 3)) & 0x07FF;

    Elrs.ch[9] = ((rdata[14] >> 0) | (rdata[15] << 8)) & 0x07FF;
    Elrs.ch[10] = ((rdata[15] >> 3) | (rdata[16] << 5)) & 0x07FF;

    Elrs.ch[1] -= 992;
    Elrs.ch[2] -= 992;
    Elrs.ch[3] -= 992;
    Elrs.ch[4] -= 992;
    Elrs.ch[10] -= 992;

    Elrs.ch[5] = Channel_optimization(Elrs.ch[5]);
    Elrs.ch[6] = Channel_optimization(Elrs.ch[6]);
    Elrs.ch[7] = Channel_optimization(Elrs.ch[7]);
    Elrs.ch[8] = Channel_optimization(Elrs.ch[8]);
    Elrs.ch[9] = Channel_optimization(Elrs.ch[9]);

    for (uint8_t i = 5; i < 10; i++)
    {
        if (Elrs.ch_last[i] != Elrs.ch[i])
            Elrs.ch_flag[i] = Elrs.ch[i];
        Elrs.ch_last[i] = Elrs.ch[i];
    }
    if (Elrs.ch[1] > -100 && Elrs.ch[1] < 100)
    {
        Elrs.ch[1] = 0;
    }
    if (Elrs.ch[2] > -100 && Elrs.ch[2] < 100)
    {
        Elrs.ch[2] = 0;
    }
    if (Elrs.ch[3] > -100 && Elrs.ch[3] < 100)
    {
        Elrs.ch[3] = 0;
    }
    if (Elrs.ch[4] > -100 && Elrs.ch[4] < 100)
    {
        Elrs.ch[4] = 0;
    }
}
