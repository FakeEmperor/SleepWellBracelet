#define UNION(x,y) ((x<<8) | y)

class Gyroscope { 
  
  size_t addr_;
  
  double angX_, angY_;

protected:
  
public:
  const static size_t DEF_ADDR = 0x68;
  Gyroscope(size_t address = Gyroscope::DEF_ADDR) : addr_ (address) {
        
  }

  void step() {
  
  }

  double angX() const {
    return angX_;
  }
  double angY() const {
    return angY_;
  }

};

// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
#include "Kalman.h"

const int MPU_addr=0x68;  // I2C address of the MPU-6050

Kalman kalmanX;
Kalman kalmanY;
byte data[14];

int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

double accXangle; // Angle calculate using the accelerometer
double accYangle;
double temp;
double gyroXangle = 180; // Angle calculate using the gyro
double gyroYangle = 180;
double compAngleX = 180; // Calculate the angle using a Kalman filter
double compAngleY = 180;
double kalAngleX; // Calculate the angle using a Kalman filter
double kalAngleY;
uint32_t timer;

void setup(){
  Wire.begin();
  Serial.begin(9600);
  i2cWrite(0x6B,0x00, true); // Disable sleep mode      
  kalmanX.setAngle(180); // Set starting angle
  kalmanY.setAngle(180);
  timer = micros();
}
void loop(){
  i2cRead(0x3B, 14, data);

  accX = UNION(data[0], data[1]); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  accY = UNION(data[2], data[3]); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ = UNION(data[4], data[5]); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tempRaw = UNION(data[6], data[7]); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX = UNION(data[8], data[9]); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY = UNION(data[10], data[11]); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ = UNION(data[12], data[13]); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  /* Calculate the angls based on the different sensors and algorithm */
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG; 
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = -((double)gyroY/131.0);
  gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
  gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
  timer = micros();
  // The accelerometer's maximum samples rate is 1kHz

  
  Serial.print("AcX = "); Serial.print(accX);
  Serial.print(" | AcY = "); Serial.print(accY);
  Serial.print(" | AcZ = "); Serial.print(accZ);
  Serial.print(" | Tmp = "); Serial.print(tempRaw/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(gyroX);
  Serial.print(" | GyY = "); Serial.print(gyroY);
  Serial.print(" | GyZ = "); Serial.print(gyroZ);
  Serial.println("");

  Serial.println();
  Serial.print("X:");
  Serial.print(kalAngleX,0);
  Serial.print(" ");
  Serial.print("Y:");
  Serial.print(kalAngleY,0);
  Serial.println(" ");
  
  delay(100);
}

void i2cWrite(uint8_t registerAddress, bool end_mode) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(registerAddress);
  Wire.endTransmission(end_mode); // Send stop
}
void i2cWrite(uint8_t registerAddress, uint8_t data, bool end_mode){
  Wire.beginTransmission(MPU_addr);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(end_mode); // Send stop
}
void i2cRead(uint8_t registerAddress, uint8_t nbytes, uint8_t *data) {
  i2cWrite(registerAddress, false);
  Wire.requestFrom(MPU_addr, nbytes); // Send a repeated start and then release the bus after reading
  for(uint8_t i = 0; i < nbytes; i++)
    data [i]= Wire.read();
}

