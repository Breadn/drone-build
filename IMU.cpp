// C++ files for function implementations of Inertial Motion Unit

// Library includes
#include <Wire.h>
#include <Arduino.h>

/* GY-521 setup/read snippets courtesy of (c) Michael Schoeffler 2017, http://www.mschoeffler.de */

const int MPU_ADDR = 0x68;              // A0 pin set LOW

int16_t accelerometer_x, accelerometer_y, accelerometer_z;  // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z;                             //               gyro          raw data
int16_t temperature;                                        //               temperature   raw data

char tmp_str[7];                        // temporary variable used in convert function
char* convert_int16_to_str(int16_t i) { // Converts int16 to string
    sprintf(tmp_str, "%6d", i);
    return tmp_str;
}

void setupIMU() {
    Wire.begin();                       // Begin I2C
    Wire.beginTransmission(MPU_ADDR);   // Begins I2C transmission to GY-521
    Wire.write(0x6B);                   // PWR_MGMT_1 register
    Wire.write(0);                      // wake up MPU
    Wire.endTransmission(true); 

    Serial.println("gX,gY,gZ");         // Serial plotter labels
}

void readStatus() {
    Serial.print("Hello world! - IMU");
}

void readIMU() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);                       // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false);            // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 7*2, true);  // request a total of 7*2=14 registers
    
    /* "Wire.read()<<8 | Wire.read();" MSB and LSB of raw data is read and bit-ORed together to be stored into variables */
    accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    temperature = Wire.read()<<8 | Wire.read();     // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
    gyro_x = Wire.read()<<8 | Wire.read();          // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    gyro_y = Wire.read()<<8 | Wire.read();          // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
    gyro_z = Wire.read()<<8 | Wire.read();          // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
    
    /* print out data */
    /*Serial.print("aX:");   Serial.print(convert_int16_to_str(accelerometer_x));  Serial.print("\t");*/
    /*Serial.print("aY:");   Serial.print(convert_int16_to_str(accelerometer_y));  Serial.print("\t");*/
    /*Serial.print("aZ:");   Serial.print(convert_int16_to_str(accelerometer_z));  Serial.print("\t");*/
    /*Serial.print("gX:");*/ Serial.print(convert_int16_to_str(gyro_x));           Serial.print("\t");
    /*Serial.print("gY:");*/ Serial.print(convert_int16_to_str(gyro_y));           Serial.print("\t");
    /*Serial.print("gZ:");*/ Serial.print(convert_int16_to_str(gyro_z));           Serial.println();
    
    // USE DELTA TIMING IN MAIN
    delay(20);
}
