#ifndef _SVPWM_ELON_H
#define _SVPWM_ELON_H

#include <string.h>
//Code area
#define __constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
typedef enum
{
    CENTER_CONFINE=0,
    MIN_CONFINE,
    MAX_CONFINE
}svpwmMode_t;
typedef struct _svPWM_
{
    float alpha_V;// the alpha axis output voltage of IPARK 
    float beta_V; // the beta axis output voltage of IPARK
    float Tu;// phase U dutycycle ratio(0~1.0)
    float Tv;// phase V dutycycle ratio(0~1.0)
    float Tw;// phase W dutycycle ratio(0~1.0)
    float Vu;//reference phase-U voltage 		
	float Vv; //reference phase-V voltage
	float Vw;// reference phase-W voltage
	float Vmax;	// max phase voltage
	float Vmin;	//min phase voltage
	float Vcomm;//common mode voltage
	float tmp1;	//temp variable
	float tmp2;//temp variable
    float invDCbusVoltage;
    svpwmMode_t svMode;
}SVPWM_obj;


#define MATH_SQRT3_OVER_TWO 0.866025403785f
#define SVPWM_DEFAULT {0.0f}

#define SVPWM_MACRO(m)\
    \
    m.tmp1=m.alpha_V*m.invDCbusVoltage*0.5f;\
    m.tmp2=m.beta_V*m.invDCbusVoltage*MATH_SQRT3_OVER_TWO;\
    \
    m.Vu=m.alpha_V*m.invDCbusVoltage;\
    m.Vv=-m.tmp1+m.tmp2;\
    m.Vw=-m.tmp1-m.tmp2;\
	if (m.Vu>m.Vv) {m.Vmax=m.Vu; m.Vmin=m.Vv;}\
	else 	   	   {m.Vmax=m.Vv; m.Vmin=m.Vu;}\
	if (m.Vw>m.Vmax) m.Vmax=m.Vw;\
	if (m.Vw<m.Vmin) m.Vmin=m.Vw;\
    m.Vcomm=0.5f*(m.Vmin+m.Vmax);\
    if(m.svMode==CENTER_CONFINE)\
    {m.Tu=m.Vu-m.Vcomm;m.Tv=m.Vv-m.Vcomm;m.Tw=m.Vw-m.Vcomm;}\
    else if(m.svMode==MIN_CONFINE)\
    {m.Tu=m.Vu-m.Vmin-0.5f;m.Tv=m.Vv-m.Vmin-0.5f;m.Tw=m.Vw-m.Vmin-0.5f;}\
    else if(m.svMode==MAX_CONFINE)\
    {m.Tu=m.Vu-m.Vmax+0.5f;m.Tv=m.Vv-m.Vmax+0.5f;m.Tw=m.Vw-m.Vmax+0.5f;}\


#endif 

