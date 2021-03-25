#include "../include/IMU.h"

IMU::IMU()
{
    mpu.verbose(false);
}

bool IMU::isConfigurationValid(ConfigurationReq* configurationReq)
{
    MPU9250Setting mpuSettings;
    Vector3D accelBias;
    Vector3D gyroBias;
    Vector3D magBias;
    Vector3D magScale;

    mpuSettings = configurationReq->mpuSettings;
    accelBias = configurationReq->accelrBias;
    gyroBias = configurationReq->gyroBias;
    magBias = configurationReq->magBias;
    magScale = configurationReq->magScale;

    ConfigurationCfm configurationCfm;           
    if(mpu.setup(MPU_ADDRESS, mpuSettings, Wire))
    {
        mpu.setAccBias(accelBias.x, accelBias.y, accelBias.z);
        mpu.setGyroBias(gyroBias.x, gyroBias.y, gyroBias.z);
        mpu.setMagBias(magBias.x, magBias.y, magBias.z);
        mpu.setMagScale(magScale.x, magScale.y, magScale.z);
        mpu.setMagneticDeclination(configurationReq->magneticDelication);
        return true;
    }
    else
    {
        return false;
    }
}

void IMU::receiveInitialMessage()
{
    bool isReady = false;
    while(!isReady)
    {
        U8 numberOfBytesToRead = Serial.available();
        if(numberOfBytesToRead)
        {
            U8* sig = new U8[numberOfBytesToRead];
            Serial.readBytes(sig, numberOfBytesToRead);

            SignalID signalID;
            memcpy(&signalID, sig, sizeof(SignalID));


            switch(signalID)
            {
                case CONFIGURATION_REQ:
                {
                    ConfigurationReq* configurationReq = reinterpret_cast<ConfigurationReq*>(sig);
                    ConfigurationCfm configurationCfm;

                    configurationCfm.isValid = isConfigurationValid(configurationReq);
                    sendSignal(configurationCfm, sizeof(ConfigurationCfm));

                    if(configurationCfm.isValid)
                    {
                        isReady = true;
                    }

                    break;
                }

                case CALIBRATION_REQ:
                {
                    CalibrationReq* calibrationReq = reinterpret_cast<CalibrationReq*>(sig);
                    CalibrationCfm* calibrationCfm = (CalibrationCfm*) calloc(1, sizeof(CalibrationCfm));
                    static U8 noOfEntry = 0;
                    calibrationCfm->isValid = true;

                    if(noOfEntry == 0)
                    {
                        if(mpu.setup(MPU_ADDRESS, calibrationReq->mpuSettings, Wire))
                        {
                            mpu.setMagneticDeclination(calibrationReq->magneticDelication);
                            calibrationCfm->isValid = true;
                        }
                        else
                        {
                            calibrationCfm->isValid = false;
                        }
                    }

                    if(calibrationCfm->isValid)
                    {
                        noOfEntry++;
                        delay(DELAY_BEFORE_CALIBRATION);

                        if(noOfEntry == 1)
                        {
                            mpu.calibrateAccelGyro();
                        }
                        else if(noOfEntry == 2)
                        {
                            mpu.calibrateMag();

                            calibrationCfm->accelrBias.x = mpu.getAccBiasX();
                            calibrationCfm->accelrBias.y = mpu.getAccBiasY();
                            calibrationCfm->accelrBias.z = mpu.getAccBiasZ();

                            calibrationCfm->gyroBias.x = mpu.getGyroBiasX();
                            calibrationCfm->gyroBias.y = mpu.getGyroBiasY();
                            calibrationCfm->gyroBias.z = mpu.getGyroBiasZ();

                            calibrationCfm->magBias.x = mpu.getMagBiasX();
                            calibrationCfm->magBias.y = mpu.getMagBiasY();
                            calibrationCfm->magBias.z = mpu.getMagBiasZ();

                            calibrationCfm->magScale.x = mpu.getMagScaleX();
                            calibrationCfm->magScale.y = mpu.getMagScaleY();
                            calibrationCfm->magScale.z = mpu.getMagScaleZ();

                            isReady = true;
                        }
                    }
                    sendSignal(calibrationCfm, sizeof(CalibrationCfm));
                    free(calibrationCfm);
                    break;
                }

                default:
                    break;
            }
            delete[] sig;
        }
    }
}

template<class Signal>
void IMU::sendSignal(Signal sig, U8 size)
{
    Serial.write((U8*)&sig, size);
}

void IMU::updateIMUandSend()
{
    if (mpu.update())
    {
        static U32 prev_ms = millis();
        if (millis() > prev_ms + REFRESH_TIME)
        {
            MeasurementInd measurementInd;
            measurementInd.pitch = mpu.getPitch();
            measurementInd.yaw = mpu.getYaw();
            measurementInd.roll = mpu.getYaw();
            sendSignal(measurementInd, sizeof(MeasurementInd));
            prev_ms = millis();
        }
    }
}