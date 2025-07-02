
#ifndef _FOC_ELON_H
#define _FOC_ELON_H


#include <Arduino.h>
#include "SVPWM_Elon.h"
#include "AS5600.h"
#define PWMA 7  
#define PWMB 8
#define PWMC 25
#define ENABLEPIN 32
#define FAULTPIN 33
#define POLEPAIRS 7 // Number of pole pairs in the motor
#define DC_VOLTAGE 12.0f
#define MATH_PI_OVER_3 1.0471975512f
#define MATH_SQRT3 1.73205080757f
const int ENCODERDIR=1;
typedef struct 
{
  float kp;
  float ki;
  float kd;
  float output;
  float ref;
  float fbk;
  float outputRamp;
  float outputLast;
  float errorLast;
  float integralLast;
  float integralLimit;
  float outputLimit;
  unsigned long timeStampLast;
  unsigned char flag_antiWD; 

}pidController_t;

typedef struct 
{
  float cosine;
  float sine;
}trigValue_t;


void FOCInit(void);

float normalizeAngle(float angle);
float openloopAngleGenerate(float rpm);
float getElectricalAngle(float mechanicalAngle);
float fsat(float value, float minVal, float maxVal);
void pidInit(pidController_t *obj,float kp,float ki,float kd,float ramp, float limit);
float pidRun(pidController_t *obj);
void setPWM(float Ta, float Tb, float Tc);
void trigCalculate(trigValue_t *obj,float electricalAngle);
void setTorqueMacro(trigValue_t *obj,float Uq,float Ud);
void sensorAlign();
void faultDetection();








#endif 

