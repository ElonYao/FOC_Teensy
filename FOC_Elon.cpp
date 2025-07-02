#include "WireIMXRT.h"

#include "core_pins.h"
#include "FOC_Elon.h"


SVPWM_obj svpwm1=SVPWM_DEFAULT;

//Magnetic encoder 
Sensor_AS5600 Motor1=Sensor_AS5600(0);
TwoWire Motor1_i2c=Wire;

//Control Macros
#define MOTORENABLE digitalWrite(ENABLEPIN, HIGH)
#define MOTORDISABLE digitalWrite(ENABLEPIN, LOW)

float zeroElectricalAngle = 0.0f; // Zero electrical angle for the motor

void FOCInit(void) 
{
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(PWMC, OUTPUT);
    pinMode(ENABLEPIN,OUTPUT);
    pinMode(FAULTPIN,INPUT);
    MOTORENABLE;
    // Initialize PWM
    analogWriteFrequency(PWMA, 20000);  // Set frequency to 20kHz
    analogWriteFrequency(PWMB, 20000);
    analogWriteFrequency(PWMC, 20000);
    //Initialize magnetic encoder
    Motor1.Sensor_init(&Motor1_i2c);
    svpwm1.svMode=MAX_CONFINE;
}
float normalizeAngle(float angle) 
{
    float a = angle-(int)(angle / (2 * PI)) * (2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}
float getElectricalAngle(float mechanicalAngle) 
{
    // Convert mechanical angle to electrical angle
    return normalizeAngle(ENCODERDIR*mechanicalAngle * POLEPAIRS - zeroElectricalAngle);
}


float openloopAngleGenerate(float rpm) 
{
    // Generate an open-loop angle based on frequency
    static unsigned long timeStampLast = 0;
    static float angle = 0.0f;
    unsigned long timeStamp=micros();
    float Ts=(timeStamp-timeStampLast)*1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    angle+= rpm/60.0f * Ts * 2 * PI;// Normalize angle to [0, 2*PI]
    angle= normalizeAngle(angle);
    timeStampLast = timeStamp;
    return angle;// Return mechanical angle in radians
}

float fsat(float value, float minVal, float maxVal)
{
    return max(minVal, min(value, maxVal));
}
void pidInit(pidController_t *obj,float kp,float ki,float kd,float ramp, float limit)
{
    obj->kp=kp;
    obj->ki=ki;
    obj->kd=kd;
    obj->ref=obj->fbk=0;
    obj->outputRamp=ramp;
    obj->outputLimit=obj->integralLimit=limit;
    obj->flag_antiWD=1;
    obj->outputLast=obj->errorLast=obj->integralLast=0;
    obj->timeStampLast=micros();
}
float pidRun(pidController_t *obj)
{
    unsigned long timeStamp=micros();
    float Ts=(timeStamp-obj->timeStampLast)*1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    float error=obj->ref-obj->fbk;
    // Integral
    float integral = obj->integralLast + obj->ki*Ts*0.5f*(error + obj->errorLast);
    if(obj->flag_antiWD)
    {
        integral = constrain(integral, -obj->integralLimit, obj->integralLimit);
    }
    
    // derivative
    float derivative = obj->kd*(error - obj->errorLast)/Ts;

    // output
    float output = obj->kp*error + integral + derivative;
    output = constrain(output, -obj->outputLimit, obj->outputLimit);

    if(obj->outputRamp > 0){
        // contstrain the output ramp
        float output_rate = (output - obj->outputLast )/Ts;
        if (output_rate > obj->outputRamp)
            output = obj->outputLast + obj->outputRamp*Ts;
        else if (output_rate < -obj->outputRamp)
            output = obj->outputLast - obj->outputRamp*Ts;
    }
    // Save values
    obj->integralLast = integral;
    obj->outputLast = output;
    obj->errorLast = error;
    obj->timeStampLast = timeStamp;
    return output;
}
//lower the computation load
void trigCalculate(trigValue_t *obj,float electricalAngle)
{
    obj->cosine=cos(electricalAngle);
    obj->sine=sin(electricalAngle);
}
void setPWM(float Ta, float Tb, float Tc) 
{
    float dcA = fsat(-Ta, -0.5f, 0.5f)+0.5f;
    float dcB = fsat(-Tb, -0.5f, 0.5f)+0.5f;
    float dcC = fsat(-Tc, -0.5f, 0.5f)+0.5f;
    analogWrite(PWMA, (int)(dcA * 255));
    analogWrite(PWMB, (int)(dcB * 255));
    analogWrite(PWMC, (int)(dcC * 255));
}
void setTorqueMacro(trigValue_t *obj,float Uq,float Ud)
{
  //Inverse park
  svpwm1.alpha_V= -Uq*obj->sine+Ud*obj->cosine; 
  svpwm1.beta_V= Uq*obj->cosine+Ud*obj->sine; 

  svpwm1.invDCbusVoltage=1/DC_VOLTAGE;

  SVPWM_MACRO(svpwm1)
  setPWM(svpwm1.Tu,svpwm1.Tv,svpwm1.Tw);
  /*
  //Simulation code
  float Ua = DC_VOLTAGE*(fsat(-svpwm.Tu,-0.5f,0.5f)+0.5f);
  float Ub = DC_VOLTAGE*(fsat(-svpwm.Tv,-0.5f,0.5f)+0.5f);
  float Uc = DC_VOLTAGE*(fsat(-svpwm.Tw,-0.5f,0.5f)+0.5f);
  printPWM(Ua,Ub,Uc);
  */
}
//Sensor alignment
void sensorAlign()
{ 
   trigValue_t temp;
   temp.cosine=cos(1.5*PI);
   temp.sine=sin(1.5*PI);
   setTorqueMacro(&temp,0.5f,0);
  delay(1000);
  Motor1.Sensor_update();
  zeroElectricalAngle=normalizeAngle(ENCODERDIR*Motor1.getMechanicalAngle() * POLEPAIRS - zeroElectricalAngle);
  setTorqueMacro(&temp,0,0);
  Serial.print("zeroElectricalAngle：");
  Serial.println(zeroElectricalAngle);
}
void faultDetection()
{
    //if falut pin was pulled down 
    static int faultCounter=0;
    if(!digitalRead(FAULTPIN))
    {
        faultCounter++;
        if(faultCounter>=2000) MOTORDISABLE;
    }
    else
    {
        if(faultCounter--<0) faultCounter=0;
    }

}


