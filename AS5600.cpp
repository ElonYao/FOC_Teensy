#include "AS5600.h"

float Sensor_AS5600::getSensorAngle() {
  
  byte readArray[2];
  uint32_t readValue = 0;

  wire->beginTransmission(0x36);
  wire->write(0x0C);
  wire->endTransmission(false);

  wire->requestFrom(0x36,2);
  for (byte i=0; i < 2; i++) {
    readArray[i] = wire->read();
  }
    readValue|=readArray[0];
    readValue<<=8;
    readValue|=readArray[1];
  return (float)readValue*RAW2RAD;

}

Sensor_AS5600::Sensor_AS5600(int Mot_Num) {
   _Mot_Num=Mot_Num; 
   
}

void Sensor_AS5600::Sensor_init(TwoWire* _wire) {
    wire=_wire;
    wire->setClock(400000U);
    wire->begin(); 
    delay(500);
    getSensorAngle(); 
    delayMicroseconds(1);
    vel_angle_prev = getSensorAngle(); 
    vel_angle_prev_ts = micros();
    delay(1);
    getSensorAngle(); 
    delayMicroseconds(1);
    angle_prev = getSensorAngle(); 
    angle_prev_ts = micros();
}

void Sensor_AS5600::Sensor_update() {
    float val = getSensorAngle();
    angle_prev_ts = micros();
    float d_angle = val - angle_prev;
    // Turn counter
    if(abs(d_angle) > (0.8f*TWO_PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
}

float Sensor_AS5600::getMechanicalAngle() {
    return angle_prev;
}

float Sensor_AS5600::getAngle(){
    return ((float)full_rotations * TWO_PI + angle_prev);
}

float Sensor_AS5600::getVelocity() {
    // Calculate sampling time 
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    //
    if(Ts <= 0 || Ts >=0.3f) Ts = 1e-3f;
    // Speed calculation
    float vel = ( (float)(full_rotations - vel_full_rotations)*TWO_PI + (angle_prev - vel_angle_prev) ) / Ts;    
    // Save for next cycle
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}
