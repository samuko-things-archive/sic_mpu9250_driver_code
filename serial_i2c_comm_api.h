#ifndef SERIAL_I2C_COMM_API_H
#define SERIAL_I2C_COMM_API_H
#include <Arduino.h>
#include <Wire.h>
#include "eeprom_setup.h"




////////////////////////////////////////////////

void initLed0()
{
  pinMode(A0, OUTPUT);
}
void onLed0()
{
  digitalWrite(A0, HIGH);
}
void offLed0()
{
  digitalWrite(A0, LOW);
}


void initLed1()
{
  pinMode(A1, OUTPUT);
}
void onLed1()
{
  digitalWrite(A1, HIGH);
}
void offLed1()
{
  digitalWrite(A1, LOW);
}
////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////
String resetEEPROM(){
  setFIRST_TIME(0);
  return "1";
}
////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////
String updateFilterGain(float gain){
  setFilterGain(gain);
  return "1";
}
String sendFilterGain(){
  return String(filterGain, 3);
}

String updateI2CADDRESS(int address){
  setI2CADDRESS(address);               
  return "1";
}
String sendI2CADDRESS(){
  return String(i2cAddress);
}

/////////////////////////////////////////////////////////




///////// DIFFERENT TASK FOR SERIAL AND I2C COMMUNICATION //////////

/////////////////////////////////////////////////////////////////////////////////////
String sendRawAccData(){
  String data = String(axRaw, 6);
  data += ",";
  data += String(ayRaw, 6);
  data += ",";
  data += String(azRaw, 6);
  return data;
}

String sendOffAccData(){
  String data = String(axOff, 6);
  data += ",";
  data += String(ayOff, 6);
  data += ",";
  data += String(azOff, 6);
  return data;
}

String sendCalAccData(){
  String data = String(axCal, 6);
  data += ",";
  data += String(ayCal, 6);
  data += ",";
  data += String(azCal, 6);
  return data;
}



String sendRawGyroData(){
  String data = String(gxRaw, 6);
  data += ",";
  data += String(gyRaw, 6);
  data += ",";
  data += String(gzRaw, 6);
  return data;
}

String sendOffGyroData(){
  String data = String(gxOff, 6);
  data += ",";
  data += String(gyOff, 6);
  data += ",";
  data += String(gzOff, 6);
  return data;
}

String sendCalGyroData(){
  String data = String(gxCal, 6);
  data += ",";
  data += String(gyCal, 6);
  data += ",";
  data += String(gzCal, 6);
  return data;
}


String sendRawMagData(){
  String data = String(mxRaw, 6);
  data += ",";
  data += String(myRaw, 6);
  data += ",";
  data += String(mzRaw, 6);
  return data;
}

String sendCalMagData(){
  String data = String(mxCal, 6);
  data += ",";
  data += String(myCal, 6);
  data += ",";
  data += String(mzCal, 6);
  return data;
}

String sendBvectMagData(){
  String data = String(b_vect[0], 6);
  data += ",";
  data += String(b_vect[1], 6);
  data += ",";
  data += String(b_vect[2], 6);
  return data;
}

String sendAmatR0(){
  String data = String(A_mat[0][0], 6);
  data += ",";
  data += String(A_mat[0][1], 6);
  data += ",";
  data += String(A_mat[0][2], 6);
  return data;
}

String sendAmatR1(){
  String data = String(A_mat[1][0], 6);
  data += ",";
  data += String(A_mat[1][1], 6);
  data += ",";
  data += String(A_mat[1][2], 6);
  return data;
}

String sendAmatR2(){
  String data = String(A_mat[2][0], 6);
  data += ",";
  data += String(A_mat[2][1], 6);
  data += ",";
  data += String(A_mat[2][2], 6);
  return data;
}




String sendRPY()
{
  String data = String(roll, 6);
  data += ",";
  data += String(pitch, 6);
  data += ",";
  data += String(yaw, 6);
  return data;
}

String sendQuat()
{
  String data = String(qw, 6);
  data += ",";
  data += String(qx, 6);
  data += ",";
  data += String(qy, 6);
  data += ",";
  data += String(qz, 6);
  return data;
}

String sendAngleVariance(){
  String data = String(roll_variance, 10);
  data += ",";
  data += String(pitch_variance, 10);
  data += ",";
  data += String(yaw_variance, 10);
  return data;
}

String sendRateVariance(){
  String data = String(roll_rate_variance, 10);
  data += ",";
  data += String(pitch_rate_variance, 10);
  data += ",";
  data += String(yaw_rate_variance, 10);
  return data;
}

String sendAccVariance(){
  String data = String(accx_variance, 10);
  data += ",";
  data += String(accy_variance, 10);
  data += ",";
  data += String(accz_variance, 10);
  return data;
}

///////////////////////////////////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////////////////
String updateAccOffset(float ax, float ay, float az){
  setAxOffset(ax);
  setAyOffset(ay);
  setAzOffset(az);
  return "1";
}
////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
String updateGyroOffset(float gx, float gy, float gz){
  setGxOffset(gx);
  setGyOffset(gy);
  setGzOffset(gz);
  return "1";
}

////////////////////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////////////////
String updateBvect(float bx, float by, float bz){
  setBvectX(bx);
  setBvectY(by);
  setBvectZ(bz);
  return "1";
}
////////////////////////////////////////////////////////////////////////






///////////////////////////////////////////////////////////////////////
String updateAmatR0(float amat0, float amat1, float amat2){
  setAmat00(amat0);
  setAmat01(amat1);
  setAmat02(amat2);
  return "1";
}

String updateAmatR1(float amat0, float amat1, float amat2){
  setAmat10(amat0);
  setAmat11(amat1);
  setAmat12(amat2);
  return "1";
}

String updateAmatR2(float amat0, float amat1, float amat2){
  setAmat20(amat0);
  setAmat21(amat1);
  setAmat22(amat2);
  return "1";
}
///////////////////////////////////////////////////////////////////






///////////////////////////////////////////////////////////////////
String updateAngleVariance(float r, float p, float y){
  setRollAngleVariance(r);
  setPitchAngleVariance(p);
  setYawAngleVariance(y);
  return "1";
}
//////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////
String updateRateVariance(float r, float p, float y){
  setRollRateVariance(r);
  setPitchRateVariance(p);
  setYawRateVariance(y);
  return "1";
}
//////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
String updateAccVariance(float ax, float ay, float az){
  setAccxVariance(ax);
  setAccyVariance(ay);
  setAcczVariance(az);
  return "1";
}
////////////////////////////////////////////////////////////////////















///////////////// SERIAL COMMUNICATION //////////////////////
String ser_msg = "";

String serMsg = "", serMsgBuffer, serDataBuffer[4];

void serialReceiveAndSendData()
{
  int indexPos = 0, i = 0;

  if (Serial.available() > 0)
  {
    while (Serial.available())
    {
      serMsg = Serial.readString();
    }
    serMsg.trim();
    if (serMsg != "")
    {
      do
      {
        indexPos = serMsg.indexOf(',');
        if (indexPos != -1)
        {
          serMsgBuffer = serMsg.substring(0, indexPos);
          serMsg = serMsg.substring(indexPos + 1, serMsg.length());
          serDataBuffer[i] = serMsgBuffer;
          serMsgBuffer = "";
        }
        else
        {
          if (serMsg.length() > 0)
            serDataBuffer[i] = serMsg;
        }
        i += 1;
      } while (indexPos >= 0);
    }

    if (serDataBuffer[0] != "")
    {
      onLed1();

      /////////////// FUNCTION CALLS /////////////////////

      if (serDataBuffer[0] == "acc-raw")
      {
        ser_msg = sendRawAccData();
      }
      else if (serDataBuffer[0] == "acc-off") {
        if (serDataBuffer[1]=="") ser_msg = sendOffAccData();
        else ser_msg = updateAccOffset(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }
      else if (serDataBuffer[0] == "acc-cal")
      {
        ser_msg = sendCalAccData();
      }


      else if (serDataBuffer[0] == "gyro-raw")
      {
        ser_msg = sendRawGyroData();
      }
      else if (serDataBuffer[0] == "gyro-off") {
        if (serDataBuffer[1]=="") ser_msg = sendOffGyroData();
        else ser_msg = updateGyroOffset(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }
      else if (serDataBuffer[0] == "gyro-cal")
      {
        ser_msg = sendCalGyroData();
      }


      else if (serDataBuffer[0] == "mag-raw")
      {
        ser_msg = sendRawMagData();
      }
      else if (serDataBuffer[0] == "mag-cal")
      {
        ser_msg = sendCalMagData();
      }

      else if (serDataBuffer[0] == "bvect") {
        if (serDataBuffer[1]=="") ser_msg = sendBvectMagData();
        else ser_msg = updateBvect(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }


      else if (serDataBuffer[0] == "amatR0") {
        if (serDataBuffer[1]=="") ser_msg = sendAmatR0();
        else ser_msg = updateAmatR0(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "amatR1") {
        if (serDataBuffer[1]=="") ser_msg = sendAmatR1();
        else ser_msg = updateAmatR1(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "amatR2") {
        if (serDataBuffer[1]=="") ser_msg = sendAmatR2();
        else ser_msg = updateAmatR2(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }



      else if (serDataBuffer[0] == "gyro-var") {
        if (serDataBuffer[1]=="") ser_msg = sendRateVariance();
        else ser_msg = updateRateVariance(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }



      else if (serDataBuffer[0] == "acc-var") {
        if (serDataBuffer[1]=="") ser_msg = sendAccVariance();
        else ser_msg = updateAccVariance(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }



      else if (serDataBuffer[0] == "reset") {
        ser_msg = resetEEPROM();
      }



      else if (serDataBuffer[0] == "rpy")
      {
        ser_msg = sendRPY();
      }
      else if (serDataBuffer[0] == "quat")
      {
        ser_msg = sendQuat();
      }
 
      else if (serDataBuffer[0] == "rpy-var") {
        if (serDataBuffer[1]=="") ser_msg = sendAngleVariance();
        else ser_msg = updateAngleVariance(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "gain") {
        if (serDataBuffer[1]=="") ser_msg = sendFilterGain();
        else ser_msg = updateFilterGain(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }
      else if (serDataBuffer[0] == "i2c") {
        if (serDataBuffer[1]=="") ser_msg = sendI2CADDRESS();
        else ser_msg = updateI2CADDRESS(constrain(serDataBuffer[1].toInt(), 0, 127));
        Serial.println(ser_msg);
        ser_msg = "";
      }


      Serial.println(ser_msg);
      ser_msg = "";
      offLed1();

      ////////////////////////////////////////////////////
    }
    else
    {
      ser_msg = "0";
      Serial.println(ser_msg);
      ser_msg = "";
    }
  }

  serMsg = "";
  serMsgBuffer = "";
  serDataBuffer[0] = "";
  serDataBuffer[1] = "";
  serDataBuffer[2] = "";
  serDataBuffer[3] = "";
}
//////////////////////////////////////////////////////////////////////////







//////////////////////// I2C COMMUNICATION ////////////////////////////////////

String i2c_msg = "";

String i2cMsg = "", i2cMsgBuffer, i2cDataBuffer[5];

void i2cSlaveSendData()
{
  String msg = "";
  if (i2c_msg != "")
  {
    msg = i2c_msg;
    i2c_msg = "";
  }
  else
  {
    msg = "0";
    i2c_msg = "";
  }
  char charArray[msg.length() + 1];
  msg.toCharArray(charArray, msg.length() + 1);
  Wire.write(charArray, msg.length());
}

void i2cSlaveReceiveData(int dataSizeInBytes)
{
  onLed0();

  int indexPos = 0, i = 0;

  for (int i = 0; i < dataSizeInBytes; i += 1)
  {
    char c = Wire.read();
    i2cMsg += c;
  }

  i2cMsg.trim();

  if (i2cMsg != "")
  {
    do
    {
      indexPos = i2cMsg.indexOf(',');
      if (indexPos != -1)
      {
        i2cMsgBuffer = i2cMsg.substring(0, indexPos);
        i2cMsg = i2cMsg.substring(indexPos + 1, i2cMsg.length());
        i2cDataBuffer[i] = i2cMsgBuffer;
        i2cMsgBuffer = "";
      }
      else
      {
        if (i2cMsg.length() > 0)
          i2cDataBuffer[i] = i2cMsg;
      }
      i += 1;
    } while (indexPos >= 0);
  }

  if (i2cDataBuffer[0] == "rpy")
  {
    i2c_msg = sendRPY();
  }

  else if (i2cDataBuffer[0] == "quat")
  {
    i2c_msg = sendRPY();
  }

  else if (i2cDataBuffer[0] == "rpy-rate")
  {
    i2c_msg = sendCalGyroData();
  }

  else if (i2cDataBuffer[0] == "acc-cal")
  {
    i2c_msg = sendCalAccData();
  }

  else if (i2cDataBuffer[0] == "gyro-var")
  {
    i2c_msg = sendRateVariance();
  }

  else if (i2cDataBuffer[0] == "rpy-var")
  {
    i2c_msg = sendAngleVariance();
  }

  else if (i2cDataBuffer[0] == "acc-var")
  {
    i2c_msg = sendAccVariance();
  }

  i2cMsg = "";
  i2cMsgBuffer = "";
  i2cDataBuffer[0] = "";
  i2cDataBuffer[1] = "";
  i2cDataBuffer[2] = "";
  i2cDataBuffer[3] = "";
  i2cDataBuffer[4] = "";

  offLed0();
}

/////////////////////////////////////////////////////////


#endif