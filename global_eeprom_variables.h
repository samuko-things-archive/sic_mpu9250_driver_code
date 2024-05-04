#ifndef GLOBAL_EEPROM_VARIABLES
#define GLOBAL_EEPROM_VARIABLES

#include<Wire.h>
#include "madgwick_filter.h"

MadgwickFilter madgwickFilter;

int mode = 2; // 1 - calibration, 2 - madgwickfilter


///////////////////////////////////////////////////
float axRaw = 0.00;
float ayRaw = 0.00;
float azRaw = 0.00;

float axCal = 0.00;
float ayCal = 0.00;
float azCal = 0.00;

float axOff = 0.00;
float ayOff = 0.00;
float azOff = 0.00;
////////////////////////////////////////////////////


////////////////////////////////////////////////////
float gxRaw = 0.00;
float gyRaw = 0.00;
float gzRaw = 0.00;

float gxCal = 0.00;
float gyCal = 0.00;
float gzCal = 0.00;

float gxOff = 0.00;
float gyOff = 0.00;
float gzOff = 0.00;
////////////////////////////////////////////////////


////////////////////////////////////////////////////
// magCal = A*(magRaw - b) using the A matrix and b vector to remove the magnetic offsets
float mxRaw = 0.00;
float myRaw = 0.00;
float mzRaw = 0.00;

float mxCal = 0.00;
float myCal = 0.00;
float mzCal = 0.00;

float A_mat[3][3];
float b_vect[3];
float mag_vect[3];
////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
float roll=0.00;
float pitch=0.00;
float yaw=0.00;

float qw=0.00;
float qx=0.00;
float qy=0.00;
float qz=0.00;
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
float roll_variance = 0.00;
float pitch_variance = 0.00;
float yaw_variance = 0.00;

float roll_rate_variance = 0.00;
float pitch_rate_variance = 0.00;
float yaw_rate_variance = 0.00;

float accx_variance = 0.00;
float accy_variance = 0.00;
float accz_variance = 0.00;
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
int i2cAddress = 104; // 0x68
float filterGain = 0.1;
////////////////////////////////////////////////////////////////////////////


#endif


