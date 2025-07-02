#include "FOC_CAN.h"
#include <FlexCAN_T4.h>
#include "AS5600.h"
#include "FOC_Elon.h"
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CAN;

CAN_message_t MasterMotor;
uint32_t sendTimer=0;
float positionAngle=0;
extern Sensor_AS5600 Motor1;
extern pidController_t PD_angle;
static void canParse(const CAN_message_t &msg) 
{
  uint32_t temp=0;
  temp|=msg.buf[3];
  temp<<=8;
  temp|=msg.buf[2];
  positionAngle=(float)temp*0.0030518f-100;
}
void CANinit()
{
  //CAN initial part
  CAN.begin();
  CAN.setBaudRate(500000);
  CAN.setMaxMB(2);
  CAN.setMB((FLEXCAN_MAILBOX)0,RX,STD);
  CAN.setMB((FLEXCAN_MAILBOX)1,TX,STD);

  CAN.setMBFilter(REJECT_ALL);
  CAN.enableMBInterrupts();
  CAN.onReceive(MB0,canParse);
  CAN.setMBFilter(MB0,0x100);
  CAN.mailboxStatus();
  MasterMotor.id =0x101;
  //MasterMotor.buf[8]={0};
  sendTimer = millis();
}
float getComAngle()
{
  return positionAngle;
}
void canUpdate()
{
  uint16_t temp=0;
  if ( millis() - sendTimer >= 20 ) {

    temp=(uint16_t)((PD_angle.fbk+100)*327.675f);
    MasterMotor.buf[2]=temp & 0xFF;
    MasterMotor.buf[3]=(temp>>8)& 0xFF;
    CAN.write(MasterMotor);
    sendTimer = millis();
  }
}