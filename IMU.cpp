// C++ file for function definitions of Inertial Motion Unit
/* GY-521 setup & IMU_error function courtesy of (c) 2019 Dejan, https://howtomechatronics.com */

    /**
     * MPU General Architecture:
     *  - Individual 16-bit measurements are stored in 2 registers (1-byte wide), each with a MSB and LSB
     *      + .read() returns the first byte of the uint16 measurement in buffer, then points to next byte
     *      + To read entire measurement, the first byte (MSB) is shifted into var, then second byte (LSB) bitwise-ORed into var
     *  - (uint16_t) cast to guard against bitwise shift signed int overflow
     * 
     * MPU Config:
     *  - Default sensitivity configuration of accelerometer is +/- 2g
     *  - Default sensitivity configuration of gyroscope is +/- 250deg/s
     **/


// Library includes
#include <Wire.h>
#include <Arduino.h>                    // For use on default Arduino IDE

// User includes
#include "IMU.h"
#include "MathTools.h"



const int MPU_ADDR = 0x68;              // A0 pin set LOW
const double GYRO_VOLTAGE = 5.0;
const double NULL_VOLTAGE = 0.0;
const double ACC_SEN = 16384.0;                 // Sensitivity for +/- 2 g
const double GYRO_SEN = 131.0;                  // Sensitivity for +/- 250 deg/s
const int CALIBRATION_QUALITY = 5000;            // Number of samples to take for calibration

int16_t accelRaw_x, accelRaw_y, accelRaw_z;     // variables for accelerometer raw data
int16_t gyroRaw_x, gyroRaw_y, gyroRaw_z;        //               gyroscope     raw data
int16_t tempRaw;                                //               temperature   raw data
double accel_x, accel_y, accel_z;               // variables for accelerometer data
double gyro_x, gyro_y, gyro_z;                  //               gyroscope     data
double temp;                                    //               temperature   data
double accelErr_x, accelErr_y, accelErr_z = 0;  // variables for accelerometer error
double gyroErr_x, gyroErr_y, gyroErr_z = 0;     //               gyroscope     error
double ang_x, ang_y, ang_z = 0;                 // variables for angle


void statusIMU() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75);
    Wire.endTransmission(true);
    Wire.requestFrom(MPU_ADDR, 1, true);
    if(Wire.available()) {
        Serial.print("STATUS: OK | WHOAMI: 0x"); Serial.println(Wire.read(), HEX);
    }
    else {
        Serial.print("STATUS: N/OK");
    }
}

void beginIMU() {
    Wire.begin();                       // Begin I2C
    Wire.beginTransmission(MPU_ADDR);   // Specify I2C transmission to device address (MPU-6050)
    Wire.write(0x6B);                   // PWR_MGMT_1 register (SLEEP)
    Wire.write(0x00);                   // Wake up MPU
    Wire.endTransmission();             // End transmission  

    calibrateIMU();
    delay(1000);

    //Serial.println("gX,gY,gZ");         // Serial plotter labels
}

void readIMU() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);                       // point at address 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false);            // False parameter indicates master will not release bus, allows master to send multiple transmissions to slave
    Wire.requestFrom(MPU_ADDR, 7*2, true);  // Master requests a total of 7*2=14 registers starting from MPU_ADDR (7 pairs for 7 measurements). True parameter releases bus
    
    accelRaw_x = (uint16_t) Wire.read()<<8;    // reading registers: 0x3B (ACCEL_XOUT_H) 
    accelRaw_x |= Wire.read();                 //                    0x3C (ACCEL_XOUT_L)
    accelRaw_y = (uint16_t) Wire.read()<<8;    // reading registers: 0x3D (ACCEL_YOUT_H)
    accelRaw_y |= Wire.read();                 //                    0x3E (ACCEL_YOUT_L)
    accelRaw_z = (uint16_t) Wire.read()<<8;    // reading registers: 0x3F (ACCEL_ZOUT_H)
    accelRaw_z |= Wire.read();                 //                    0x40 (ACCEL_ZOUT_L)
    // For a range of +-2g, divide raw values by 16384, according to the datasheet
    accel_x = accelRaw_x / ACC_SEN - accelErr_x;
    accel_y = accelRaw_y / ACC_SEN - accelErr_y;
    accel_z = accelRaw_z / ACC_SEN - accelErr_z;


    tempRaw = (uint16_t) Wire.read()<<8;       // reading registers: 0x41 (TEMP_OUT_H)
    tempRaw |= Wire.read();                    //                    0x42 (TEMP_OUT_L)
    // Equation from documentaion [MPU-6000/MPU-6050 Register Map and Description, p.30]
    temp = tempRaw/340.00+36.53;

    gyroRaw_x = (uint16_t) Wire.read()<<8;     // reading registers: 0x43 (GYRO_XOUT_H)
    gyroRaw_x |= Wire.read();                  //                    0x44 (GYRO_XOUT_L)
    gyroRaw_y = (uint16_t) Wire.read()<<8;     // reading registers: 0x45 (GYRO_YOUT_H)
    gyroRaw_y |= Wire.read();                  //                    0x46 (GYRO_YOUT_L)
    gyroRaw_z = (uint16_t) Wire.read()<<8;     // reading registers: 0x47 (GYRO_ZOUT_H)
    gyroRaw_z |= Wire.read();                  //                    0x48 (GYRO_ZOUT_L)
    // For a range of 250deg/s, divide raw values by 131, according to datasheet
    gyro_x = gyroRaw_x / GYRO_SEN - gyroErr_x;
    gyro_y = gyroRaw_y / GYRO_SEN - gyroErr_y;
    gyro_z = gyroRaw_z / GYRO_SEN - gyroErr_z;
    
    // USE DELTA TIMING IN MAIN
    // delay(20);
}

void readAngle(long delta_T) {
    ang_x += gyro_x * (delta_T/1000.);
    ang_y += gyro_y * (delta_T/1000.);
    ang_z += gyro_z * (delta_T/1000.);

    ang_x = fmod(ang_x, 360.0);
    ang_y = fmod(ang_y, 360.0);
    ang_z = fmod(ang_z, 360.0);
    

}

void printIMU() {
    // Accelerometer
    /*Serial.print("aX:");   Serial.print(accelerometer_x);  Serial.print("\t");*/
    /*Serial.print("aY:");   Serial.print(accelerometer_y);  Serial.print("\t");*/
    /*Serial.print("aZ:");   Serial.print(accelerometer_z);  Serial.print("\t");*/
    // Gyroscope
    /*Serial.print("gX:");   Serial.print(gyro_x);            Serial.print("\t");*/
    /*Serial.print("gY:");   Serial.print(gyro_y);            Serial.print("\t");*/
    /*Serial.print("gZ:");   Serial.print(gyro_z);            Serial.println();*/
    // Angles
    /*Serial.print("rX:");*/ Serial.print(ang_x);           Serial.print("\t");
    /*Serial.print("rY:");*/ Serial.print(ang_y);           Serial.print("\t");
    /*Serial.print("rZ:");*/ Serial.print(ang_z);           Serial.println();
}

void calibrateIMU() {
    // Note: Keep IMU in normal position during calibration
    // Read accelerometer values 200 times
    Serial.println("Calibrating IMU... (Do not disturb!)");
    int c = 0;
    while (c < CALIBRATION_QUALITY) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 6, true);

        accelRaw_x = (uint16_t) Wire.read()<<8;    // reading registers: 0x3B (ACCEL_XOUT_H) 
        accelRaw_x |= Wire.read();                 //                    0x3C (ACCEL_XOUT_L)
        accelRaw_y = (uint16_t) Wire.read()<<8;    // reading registers: 0x3D (ACCEL_YOUT_H)
        accelRaw_y |= Wire.read();                 //                    0x3E (ACCEL_YOUT_L)
        accelRaw_z = (uint16_t) Wire.read()<<8;    // reading registers: 0x3F (ACCEL_ZOUT_H)
        accelRaw_z |= Wire.read();                 //                    0x40 (ACCEL_ZOUT_L)
        accel_x = accelRaw_x / ACC_SEN;
        accel_y = accelRaw_y / ACC_SEN;
        accel_z = accelRaw_z / ACC_SEN;
        // Sum all readings
        accelErr_x += ((atan((accel_y) / sqrt(pow((accel_x), 2) + pow((accel_z), 2))) * 180 / PI));
        accelErr_y += ((atan(-1 * (accel_x) / sqrt(pow((accel_y), 2) + pow((accel_z), 2))) * 180 / PI));
        c++;
    }
    // Average sum to get error
    accelErr_x /= CALIBRATION_QUALITY;
    accelErr_y /= CALIBRATION_QUALITY;
    
    c = 0;
    // Read gyro values 
    while (c < CALIBRATION_QUALITY) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 6, true);

        gyroRaw_x = (uint16_t) Wire.read()<<8;     // reading registers: 0x43 (GYRO_XOUT_H)
        gyroRaw_x |= Wire.read();                  //                    0x44 (GYRO_XOUT_L)
        gyroRaw_y = (uint16_t) Wire.read()<<8;     // reading registers: 0x45 (GYRO_YOUT_H)
        gyroRaw_y |= Wire.read();                  //                    0x46 (GYRO_YOUT_L)
        gyroRaw_z = (uint16_t) Wire.read()<<8;     // reading registers: 0x47 (GYRO_ZOUT_H)
        gyroRaw_z |= Wire.read();                  //                    0x48 (GYRO_ZOUT_L)
        gyro_x = gyroRaw_x / GYRO_SEN;
        gyro_y = gyroRaw_y / GYRO_SEN;
        gyro_z = gyroRaw_z / GYRO_SEN;
        // Sum all readings
        gyroErr_x += gyro_x;
        gyroErr_y += gyro_y;
        gyroErr_z += gyro_z;
        c++;
    }
    //Divide the sum by 200 to get the error value
    gyroErr_x /= CALIBRATION_QUALITY;
    gyroErr_y /= CALIBRATION_QUALITY;
    gyroErr_z /= CALIBRATION_QUALITY;

    // Print the error values on the Serial Monitor
    Serial.print("accelErr_x: ");
    Serial.println(accelErr_x);
    Serial.print("accelErr_y: ");
    Serial.println(accelErr_y);
    Serial.print("gyroErr_x: ");
    Serial.println(gyroErr_x);
    Serial.print("gyroErr_y: ");
    Serial.println(gyroErr_y);
    Serial.print("gyroErr_z: ");
    Serial.println(gyroErr_z);
    Serial.println("========== Calibration Complete! ==========");
}
