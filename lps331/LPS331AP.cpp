/*
 * @Author: Plla 1183136570@qq.com
 * @Date: 2022-06-28 14:57:24
 * @LastEditors: Plla 1183136570@qq.com
 * @LastEditTime: 2022-07-06 09:22:37
 * @FilePath: \undefinedi:\plla\Code\LPS331\lps331\LPS331AP.cpp
 * @Description: 
 * 
 * 气压计&温度计
 * 适用于Arduino：UNO | ESP32 | PICO
 * 
 * @HTTP:https://github.com/plla1981
 * 
 *                     license The MIT License (MIT)
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the 'Software'), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 * 
 * 
 * Version:
 * 
 * v0.0.1: 2022.6.28, Initial version.
 * 
 * Copyright (c) 2022 by Plla 1183136570@qq.com.
 */

#include "LPS331AP.h"


/**
 * @description: 初始化 bool
 * @param {uint8_t} address
 * @param {TwoWire} &wirePort
 * @return {0 & 1}
 */
bool LPS331AP::begin(uint8_t address,TwoWire &wirePort)
{
    _address = address;
    _i2cPort = &wirePort;

    _i2cPort->begin();
    if(!IsExist())
        return false;

    /*设置采样分辨率*/
    IIC_Write_Byte(RES_CONF,0x7A);
    /*启动*/
    IIC_Write_Byte(CTRL_REG1,0x80);
    return true;
}

/**
 * @description: 初始化 void
 * @param {uint8_t} address
 * @param {TwoWire} &wirePort
 * @return {0 & 1}
 */
// void LPS331AP::begin(uint8_t address,TwoWire &wirePort)
// {
//     _address = address;
//     _i2cPort = &wirePort;

//     _i2cPort->begin();
//     if(IsExist())
//     {
//         *config = true;

//         /*设置采样分辨率*/
//         IIC_Write_Byte(RES_CONF,0x7A);
//         /*启动*/
//         IIC_Write_Byte(CTRL_REG1,0x80);
//     }else
//     {
//         *config = false;
//         temperature = 0;
//         pressure = 0;
//     }
// }

/**
 * @description: IIC写数据
 * @param {uint8_t} reg
 * @param {uint8_t} data
 * @return {*}
 */
void LPS331AP::IIC_Write_Byte(uint8_t reg,uint8_t data)
{
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(reg);
    _i2cPort->write(data);
    _i2cPort->endTransmission();
}

/**
 * @description: IIC读数据
 * @param {uint8_t} reg
 * @param {uint8_t*} buf
 * @param {int} lenght
 * @return {*}
 */
void LPS331AP::IIC_Read_Byte(uint8_t reg,uint8_t* buf,int lenght)
{
    uint8_t i = 0;
    _i2cPort->beginTransmission(_address);
    reg |= 0x80; //turn auto-increment bit on
    _i2cPort->write(reg);
    _i2cPort->endTransmission(false);
    _i2cPort->requestFrom(_address,lenght);

    while (_i2cPort->available() && i<lenght)
    {
        *buf = _i2cPort->read();
        buf++;
        i++;
    }
}

/**
 * @description: 检测是否读取到设备ID
 * @return {*}
 */
bool LPS331AP::IsExist()
{
    uint8_t config_ID;
    IIC_Read_Byte(WHO_AM_ID,&config_ID,1);

    if(config_ID == CHIP_ID)
        return true;
    else
        return false;
}

/**
 * @description: 数据进行转换输出
 * @param {int32_t} *pressure
 * @param {int16_t} *temperature
 * @return {*}
 */
void LPS331AP::_dataConversion(int32_t P_DATA,int16_t T_DATA,float *P_RESULT,float *T_RESULT)
{
    float temp_data;
    int32_t temp_data_p;
    int16_t temp_data_t;

    /*calculate pressure data*/
    /*check negative or positive [bit24]*/
    if(P_DATA & 0x800000)
    {
        /* negative */
        temp_data_p = ((~P_DATA) & 0xffffff);     /* 24bit */
        temp_data_p += 1;
        temp_data = -((float) temp_data_p);
        temp_data = temp_data / 4096;
    }
    else
    {
        /*postivie*/
        temp_data = (float)P_DATA;
        temp_data = temp_data / 4096;
    }
    *P_RESULT = temp_data;

    /*calculate temperature data*/
    /*check negative or postivie [bit16]*/
    if(T_DATA & 0x8000)
    {
        /* negative */
        temp_data_t = ((~T_DATA) & 0xffff);     /* 16bit */
        temp_data_t += 1;
        temp_data = -(float)temp_data_t;
        temp_data = temp_data / 480 + 42.5;
    }
    else
    {
        /* postivie */
        temp_data = (float)T_DATA;
        temp_data = temp_data / 480 + 42.5;
    }
    *T_RESULT = temp_data;

}

/**
 * @description: 获取数据
 * @return {*}
 */
void LPS331AP::measure(void)
{
    uint8_t buff[6];
    uint8_t buff2[8];
    uint16_t T_RAW;
    uint32_t P_RAW;
    float P_RESULT,T_RESULT;

    IIC_Write_Byte(CTRL_REG2,0x01);/*0x01 设置为单次采集模式*/
    IIC_Read_Byte(STATUS_REG,buff,1);/*读取状态*/
    buff2[0] = buff[0];
    if((buff2[0] & 0x03) == 0x03)
    {
        /* 读取压力值 24bit */
        IIC_Read_Byte(PRESS_OUT_XLP,buff,1); /*[7:0]*/
        buff2[1] = buff[0];
        IIC_Read_Byte(PRESS_OUT_LP,buff,1);  /*[15:8]*/
        buff2[2] = buff[0];
        IIC_Read_Byte(PRESS_OUT_H,buff,1);   /*[23:16]*/
        buff2[3] = buff[0];
        /* 读取温度值 16bit */
        IIC_Read_Byte(TEMP_OUT_L,buff,1);    /*[7:0]*/
        buff2[4] = buff[0];
        IIC_Read_Byte(TEMP_OUT_H,buff,1);    /*[15:8]*/
        buff2[5] = buff[0];

        /* turn value */
        P_RAW = (int32_t)(((int32_t)buff2[3] << 16) | ((int32_t)buff2[2] << 8 | (int32_t)buff2[1]));/*24bit*/
        T_RAW = (int16_t)(((int16_t)buff2[5] << 8) | ((int16_t)buff2[4])) & 0xffff;/*16bit*/
        /* 数据转换 */
        _dataConversion(P_RAW,T_RAW,&P_RESULT,&T_RESULT);
        temperature =  T_RESULT;
        pressure = P_RESULT;
    }
    else
    {
        temperature = 0;
        pressure = 0;
    }
}
