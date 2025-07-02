#include "FOC_Elon.h"
#include "SVPWM_Elon.h"
#include "AS5600.h"
#include "FOC_CAN.h"
#include <FlexCAN_T4.h>

IntervalTimer timer1;
float motorTarget=0;//Serial input commmand
int commaPosition;

trigValue_t theta={0};
extern Sensor_AS5600 Motor1;
extern const int ENCODERDIR;
pidController_t PD_angle;//position control
pidController_t PI_speed;//speed control
extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CAN;

void dataPrint(void)
{
    Serial.print(PI_speed.ref);
    Serial.print(",");
    Serial.print(PI_speed.fbk);
    Serial.print(",");
    Serial.println(PI_speed.outputLast);

}
void setup() {
    Serial.begin(115200);
    CANinit();
    FOCInit();
    sensorAlign();
    timer1.begin(dataPrint, 2e4);
    pidInit(&PD_angle,0.80f,0,0.007f,1000,2.0f);
    pidInit(&PI_speed,0.18f,0.1f,0,1000,2.0f);
}

void loop() {
    
    Motor1.Sensor_update();
    //PD_angle.ref=getComAngle();
    PI_speed.ref=motorTarget;//speed target should be set to zero in torque damper control mode
    Motor1.vel_filtered=Motor1.vel_filtered*0.99f+ 0.01f*Motor1.getVelocity();
    PI_speed.fbk=ENCODERDIR*Motor1.vel_filtered;
    trigCalculate(&theta,getElectricalAngle( Motor1.getMechanicalAngle() ));
    setTorqueMacro(&theta,constrain(0.15f*(PI_speed.ref-PI_speed.fbk),-1.0f,1.0f),0);//torque damper control 
    //setTorqueMacro(&theta,pidRun(&PI_speed),0);
    //setTorqueMacro(&theta,pidRun(&PD_angle),0);
    serialReceiveUserCommand();
    faultDetection();
    CAN.events();
    canUpdate();
    /*
    // SVPWM simulation
    for (int i = 0; i < 360; i++) {
        angle_el = normalizeAngle(i * PI / 180+HALF_PI);
        trigCalculate(&theta,angle_el);
        setTorqueMacro(&theta,Uq,0);
        //delay(20);
    }
    */
}
//=========================Serial port communication=========================

String serialReceiveUserCommand() {

  // a string to hold incoming data
  static String received_chars;

  String command = "";

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;

    // end of user input
    if (inChar == '\n') {

      // execute the user command
      command = received_chars;

      commaPosition = command.indexOf('\n'); //find the position of the newline character
      if (commaPosition != -1)                
      {
        motorTarget = command.substring(0, commaPosition).toFloat(); 
        Serial.println(motorTarget);
      }
      // reset the command buffer
      received_chars = "";
    }
  }
  return command;
}