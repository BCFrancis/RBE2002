#include "imu.h"

LSM6 imu;

void IMU_sensor::Init() {
    Wire.begin();
    if (!imu.init()) {
        while (1) {
            Serial.println("Failed to detect the LSM6.");
            delay(100);
        }
    }
    imu.setFullScaleAcc(imu.ACC_FS2);
    imu.enableDefault();
}

IMU_sensor::acceleration_data IMU_sensor::ReadAcceleration() {
    imu.read();
    return {imu.a.x, imu.a.y, imu.a.z};
}

void IMU_sensor::PrintAcceleration() {
    ReadAcceleration();
    snprintf_P(report, sizeof(report),
               PSTR("A: %10d %10d %10d"),
               imu.a.x, imu.a.y, imu.a.z);
    //imu.g.x, imu.g.y, imu.g.z);
    Serial.println(report);
}