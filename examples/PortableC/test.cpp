#include <unistd.h>
#include <iostream>
#include <memory>
#include <math.h>

#include "ICM_20948_DMP.h"
#include "ICM_20948.h"
#include "i2c_wrapper.hpp"


ICM_20948_ACCEL_CONFIG_FS_SEL_e acc_fss_enum_ = gpm4;
ICM_20948_GYRO_CONFIG_1_FS_SEL_e gyro_fss_enum_ = dps2000;
int acc_fss_ = 4;
int gyro_fss_ = 2000;
int dmp_rate_ = 55;
int odr_ = 11;
std::shared_ptr<ICM_20948_I2CDEV> myICM;

void initialise(std::shared_ptr<ICM_20948_I2CDEV> myICM) {
  myICM->enableDebugging(&std::cout); // Enable debug messages to the console

  usleep(100000); // Sleep for 10ms to allow the ICM-20948 to power up before we start communicating with it
  myICM->startupDefault(true);
  usleep(100000); // Sleep for 10ms to allow the ICM-20948 to complete its startup before we start communicating with it
  auto result = myICM->checkID();
  if (result != ICM_20948_Stat_Ok) {
    throw std::runtime_error("Failed to connect to ICM-20948");
  }
  std::cout << "Successfully connected to ICM-20948";
  bool success = true;
  success &= (myICM->initializeDMP(acc_fss_enum_,gyro_fss_enum_) == ICM_20948_Stat_Ok);
  success &= (myICM->enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  success &= (myICM->enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM->enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (myICM->enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);
  success &= (myICM->enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);
  //success &= (myICM->enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED) == ICM_20948_Stat_Ok);

  //Set ODRs
  int odr_divisor = (dmp_rate_ / odr_) - 1;
  success &= (myICM->setDMPODRrate(DMP_ODR_Reg_Quat9, odr_divisor) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM->setDMPODRrate(DMP_ODR_Reg_Accel, odr_divisor) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM->setDMPODRrate(DMP_ODR_Reg_Gyro, odr_divisor) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM->setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, odr_divisor) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM->setDMPODRrate(DMP_ODR_Reg_Cpass, odr_divisor) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM->setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, odr_divisor) == ICM_20948_Stat_Ok); // Set to the maximum
  // Enable the FIFO
  success &= (myICM->enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM->enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM->resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM->resetFIFO() == ICM_20948_Stat_Ok);
  if (!success) {
    throw std::runtime_error("Failed to initialize DMP");
  }
  std::cout << "Successfully initialized DMP";
}

void read_data(std::shared_ptr<ICM_20948_I2CDEV> myICM) {
    // Read any DMP data waiting in the FIFO
    // Note:
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
    //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
    //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
    icm_20948_DMP_data_t data;
  
    do {
      myICM->readDMPdataFromFIFO(&data);

      if ((myICM->status == ICM_20948_Stat_Ok) || (myICM->status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
      {
        if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
        {
          // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
          // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
          // The quaternion data is scaled by 2^30.
          // Scale to +/- 1
          double q1 = ((double)data.Quat9.Data.Q1) / (1 << 30); // Convert to double. Divide by 2^30
          double q2 = ((double)data.Quat9.Data.Q2) / (1 << 30); // Convert to double. Divide by 2^30
          double q3 = ((double)data.Quat9.Data.Q3) / (1 << 30); // Convert to double. Divide by 2^30
          double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
        } else {
          printf("No orientation data; header: 0x%04X", data.header);
        }
        if (data.header & DMP_header_bitmap_Accel) {
          double ax = ((double)data.Raw_Accel.Data.X) * 9.80665 * acc_fss_ / 32768.0; // Convert to m/s^2
          double ay = ((double)data.Raw_Accel.Data.Y) * 9.80665 * acc_fss_ / 32768.0; // Convert to m/s^2
          double az = ((double)data.Raw_Accel.Data.Z) * 9.80665 * acc_fss_ / 32768.0; // Convert to m/s^2
        } else {
          printf("No accelerometer data; header: 0x%04X", data.header);
        }
        if (data.header & DMP_header_bitmap_Gyro) {
          double gx = ((double)(data.Raw_Gyro.Data.X)) * (M_PI / 180.0) * gyro_fss_ / 32768.0; // Convert to rad/s
          double gy = ((double)(data.Raw_Gyro.Data.Y)) * (M_PI / 180.0) * gyro_fss_ / 32768.0; // Convert to rad/s
          double gz = ((double)(data.Raw_Gyro.Data.Z)) * (M_PI / 180.0) * gyro_fss_ / 32768.0; // Convert to rad/s
        } else {
          printf("No gyroscope data; header: 0x%04X", data.header);
        }
        if (data.header & DMP_header_bitmap_Compass_Calibr) {
          printf("Received compass data! Header: 0x%04X", data.header);
          double mx = ((double)(data.Compass_Calibr.Data.X)); // Convert to uT
          double my = ((double)(data.Compass_Calibr.Data.Y)); // Convert to uT
          double mz = ((double)(data.Compass_Calibr.Data.Z)); // Convert to uT
          printf("Compass data is: MX:%f MY:%f MZ:%f", mx, my, mz);
        } else {
          printf("No compass data; header: 0x%04X", data.header);
        }
        if (data.header & DMP_header_bitmap_Compass) {
          printf("Received compass data! Header: 0x%04X", data.header);
          double mx = ((double)(data.Compass.Data.X)); // Convert to uT
          double my = ((double)(data.Compass.Data.Y)); // Convert to uT
          double mz = ((double)(data.Compass.Data.Z)); // Convert to uT
          printf("Compass data is: MX:%f MY:%f MZ:%f", mx, my, mz);
        } else {
          printf("No compass data; header: 0x%04X", data.header);
        }
        if (data.header & DMP_header_bitmap_Header2) {
          printf("Received Header2; data is: 0x%04X", data.header2);
          printf("Accuracies: Accel:%d Gyro:%d Cpass:%d", data.Accel_Accuracy, data.Gyro_Accuracy, data.Compass_Accuracy);
        }
      }
    } while (myICM->status == ICM_20948_Stat_FIFOMoreDataAvail);
}


int main(int argc, char ** argv)
{
  auto icm = std::make_shared<ICM_20948_I2CDEV>(0x04, 0x68);
  initialise(icm);
  for (int i=0; i < 20; i++) {
    read_data(icm);
  }
  return 0;
}
