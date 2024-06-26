#include "invensense_mpu9250_spi.h"
#include "vectlab.h"

#include <Wire.h>
#include "serial_i2c_comm_api.h"

float MicroTeslaToTesla(float mT)
{
  return mT * 1000000;
}

/* Mpu9250 object, SPI bus, CS on pin 10 */
bfs::Mpu9250 imu(&SPI, 10);

unsigned long serialCommTime, serialCommSampleTime = 10; // ms -> (1000/sampleTime) hz
unsigned long readImuTime, readImuSampleTime = 5;        // ms -> (1000/sampleTime) hz

void setup()
{
  /* Serial to display data */
  Serial.begin(115200);
  Serial.setTimeout(4);

  initLed0();
  initLed1();

  offLed0();
  offLed1();

  //---------------- START IMU IN SPI MODE -----------------------//
  /* Start the SPI bus */
  SPI.begin();
  //----------------------------------------------------------------//

  //---------------- INITIALIZE IMU -----------------------//
  /* Initialize and configure IMU */
  if (!imu.Begin())
  {
    // Serial.println("Error initializing communication with IMU");

    while (1)
    {
    }
  }

  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19))
  {
    // Serial.println("Error configured SRD");
    while (1)
    {
    }
  }
  //----------------------------------------------------------------//

  // update global params with eeprom contents
  updateGlobalParamsFromEERPOM();
  /////////////////////////////////////////////

  Wire.begin(i2cAddress);
  Wire.onReceive(i2cReceiveDataEvent);
  Wire.onRequest(i2cSendDataEvent);

  madgwickFilter.setAlgorithmGain(filterGain);

  onLed0();
  delay(800);
  offLed0();
  delay(400);
  onLed1();
  delay(800);
  offLed1();

  serialCommTime = millis();
  readImuTime = millis();
}

void loop()
{

  ////////// using the serial communiaction API ////////////////////////
  if ((millis() - serialCommTime) >= serialCommSampleTime)
  {
    serialReceiveAndSendData();
    serialCommTime = millis();
  }
  //////////////////////////////////////////////////////////////////////

  if ((millis() - readImuTime) >= readImuSampleTime)
  {

    if (imu.Read())
    {
      //------------READ ACC DATA AND CALIBRATE---------------//
      float ax = imu.accel_x_mps2();
      float ay = imu.accel_y_mps2();
      float az = imu.accel_z_mps2();

      // convert NED frame to NWU frame
      axRaw = ax * 1.00;
      ayRaw = ay * -1.00;
      azRaw = az * -1.00;

      axCal = axRaw - axOff;
      ayCal = ayRaw - ayOff;
      azCal = azRaw - azOff;
      //------------------------------------------------------//

      //-----------READ GYRO DATA AND CALIBRATE---------------//
      float gx = imu.gyro_x_radps();
      float gy = imu.gyro_y_radps();
      float gz = imu.gyro_z_radps();

      // convert NED frame to NWU frame
      gxRaw = gx * 1.00;
      gyRaw = gy * -1.00;
      gzRaw = gz * -1.00;

      gxCal = gxRaw - gxOff;
      gyCal = gyRaw - gyOff;
      gzCal = gzRaw - gzOff;
      //-----------------------------------------------------//

      //-----------READ MAG DATA AND CALIBRATE---------------//
      float mx = imu.mag_x_ut();
      float my = imu.mag_y_ut();
      float mz = imu.mag_z_ut();

      // convert NED frame to NWU frame
      mxRaw = mx * 1.00;
      myRaw = my * -1.00;
      mzRaw = mz * -1.00;

      // magCal = A_1*(magRaw - b) using the A matrix and b vector to remove the magnetic offsets
      mag_vect[0] = mxRaw;
      mag_vect[1] = myRaw;
      mag_vect[2] = mzRaw;

      // mag_vect = mag_vect - b_vect
      mag_vect[0] = mag_vect[0] - b_vect[0];
      mag_vect[1] = mag_vect[1] - b_vect[1];
      mag_vect[2] = mag_vect[2] - b_vect[2];

      // mag_vect = A_mat * mag_vect
      vectOp.transform(mag_vect, A_mat, mag_vect);

      mxCal = mag_vect[0];
      myCal = mag_vect[1];
      mzCal = mag_vect[2];
      //---------------------------------------------------//

      //------------- APPLY MADWICK FILTER IN NWU FRAME -----------------//
      float _ax = axCal;
      float _ay = ayCal;
      float _az = azCal;

      float _gx = gxCal;
      float _gy = gyCal;
      float _gz = gzCal;

      float _mx = MicroTeslaToTesla(mxCal);
      float _my = MicroTeslaToTesla(myCal);
      float _mz = MicroTeslaToTesla(mzCal);

      madgwickFilter.madgwickAHRSupdate(_gx, _gy, _gz, _ax, _ay, _az, _mx, _my, _mz);

      madgwickFilter.getOrientationRPY(roll, pitch, yaw);
      madgwickFilter.getOrientationQuat(qw, qx, qy, qz);
      //----------------------------------------------------//
    }

    readImuTime = millis();
  }
}