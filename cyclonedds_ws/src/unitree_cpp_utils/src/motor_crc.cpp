#include "motor_crc.h"


void cpy_msg(LowCmd &dst, const unitree_go::msg::LowCmd& msg)
{
    memcpy(&dst.head[0], &msg.head[0], 2);

    dst.levelFlag=msg.level_flag;
    dst.frameReserve=msg.frame_reserve;

    memcpy(&dst.SN[0],&msg.sn[0], 8);
    memcpy(&dst.version[0], &msg.version[0], 8);

	dst.bandWidth=msg.bandwidth;


    for(int i = 0; i<20; i++)
    {
        dst.motorCmd[i].mode=msg.motor_cmd[i].mode;
        dst.motorCmd[i].q=msg.motor_cmd[i].q;
        dst.motorCmd[i].dq=msg.motor_cmd[i].dq;
        dst.motorCmd[i].tau=msg.motor_cmd[i].tau;
        dst.motorCmd[i].Kp=msg.motor_cmd[i].kp;
        dst.motorCmd[i].Kd=msg.motor_cmd[i].kd;

        memcpy(&dst.motorCmd[i].reserve[0], &msg.motor_cmd[i].reserve[0], 12);
    }

    dst.bms.off=msg.bms_cmd.off;
    memcpy(&dst.bms.reserve[0],&msg.bms_cmd.reserve[0],  3);


    memcpy(&dst.wirelessRemote[0], &msg.wireless_remote[0], 40);

    memcpy(&dst.led[0], &msg.led[0],  12);  // go2
    memcpy(&dst.fan[0], &msg.fan[0],  2);
    dst.gpio=msg.gpio;    // go2

	dst.reserve=msg.reserve;
}

uint32_t get_crc(const LowCmd &raw){
    return crc32_core((uint32_t *)&raw, (sizeof(LowCmd)>>2)-1);
}

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}