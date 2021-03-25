#pragma once

#include <Arduino.h>
#include "../CarInterface/interface.h"
#include "../MPU9250/MPU9250.h"

typedef uint8_t U8;
typedef int8_t S8;
typedef uint16_t U16;
typedef int16_t S16;
typedef uint32_t U32;
typedef int32_t S32;


class IMU
{
    public:
    IMU();
    void receiveInitialMessage();
    void updateIMUandSend();

    private:
    template<class Signal>
    void sendSignal(Signal sig, U8 size);
    const U8 MPU_ADDRESS = 0x68;
    MPU9255 mpu;
    bool isConfigurationValid(ConfigurationReq* configurationReq);
    void calibrateImu();
    const U8 REFRESH_TIME = 25; //milliseconds
    const U16 DELAY_BEFORE_CALIBRATION = 5000; //5sec
};