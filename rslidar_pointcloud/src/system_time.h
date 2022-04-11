#ifndef SYSTEM_TIME_H
#define _SYSTEM_TIME_H_
#include <stdint.h>
#include <stddef.h>
/**
 * @brief 用于时间戳的计算
 * 根据雷达手册 head的21byte -- 30byte为时间戳
 * 考虑实际存储的时间戳，由4byte的sec和4byte的nsec组成（一共8byte）
 * nsec可以对应后4byte，但是前6byte无法直接对应
 * 所以打算舍弃前两个byte(记录年 月信息)
 * 理论上会有点小问题（跨月的时候）
 * 
 * 这里等明天再问问学长吧
 * 
 */
namespace systemTime
{
    double getTimestamp(uint32_t sec, uint32_t nsec)
    {
        //算了 先不写了明天再说
    }
}

#endif