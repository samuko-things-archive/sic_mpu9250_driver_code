# sic_mpu9250_driver_code
This is a child project of the Samuko IMU Compute (`sic_mpu9250`) project. It contains the code that is uploaded into `sic_mpu9250_driver module` via an FTDI programmer.
</br>
It has two parts - `sic_mpu9250_calibration` (i.e **MODE 1**) and `sic_mpu9250_madgwick_filter` (i.e **MODE 2**).
</br>
the `sic_mpu9250_calibration` code is needed in performing all the `mode1...` setup_py_codes. you'll know if you're running the calibration code when the blue LED in on.
</br>
the `sic_mpu9250_madgwick_filter` code is needed in performing all the `mode2...` setup_py_codes and also with the different libraries and packages (ROS2, Arduino, Python or Cpp) for you specific project. you'll know if you're running the madgwick_filter code when the green LED in on.


## How to Use the Driver Code
- Ensure you have the Arduino IDE up and running on your PC

- Download (by clicking on the green Code button above) or clone the repo into your PC

- extract and copy the library dependencies to your library folder:
  > for ubuntu OS, copy to ~/Arduino/libraries folder
  > for Windows, copy to ...Documents/Arduino/libraries folder

  > NOTE: it makes use of the `invensense-imu` library by bolder flight and `arduino_matrix_vector_lab` library by samuko-things and `imu_madgwick_filter` library by samuko-things

- Open the sic_mpu9250_driver_code folder

- Open any of the `sic_mpu9250_calibration.ino` or `sic_mpu9250_madgwick_filter` code in your Arduino IDE.

- select the board - `Arduino UNO` - and PORT (for the FTDI programmer). 

- verify and upload the code the code to the `sic_mpu9250_driver module` via FTDI programmer.
