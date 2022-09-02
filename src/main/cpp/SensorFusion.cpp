// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "SensorFusion.h"
#include "PiModule.h"
#define PI 3.14159265

#include <vector>
#include <string>
#include <math.h> 
#include <wpi/numbers>


#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/ADIS16448_IMU.h>



SensorFusion::SensorFusion(rev::SparkMaxRelativeEncoder* lEncoder, rev::SparkMaxRelativeEncoder* rEncoder): lEncoder{lEncoder}, rEncoder{rEncoder} {}

double SensorFusion::driftAlgorithm(double short_sensor, double last_short_sensor, double long_sensor, double error, double corrected, double long_bound, double max_drift) {
    double d_drift = 0;
    double derror = error;
    double mag = long_sensor;

    if ((derror >= long_bound) && (corrected < mag)) { // Out of bounds + Less than
        d_drift = max_drift;
    
    } else if ((derror < long_bound) && (corrected < mag)) { // In Bounds + Less than
        d_drift = ((bound_num - derror) / bound_num) * max_drift;
    
    } if ((derror >= long_bound) && (corrected > mag)) { // Out of Bounds + More than
        d_drift = -max_drift;
    
    } else if ((derror < long_bound) && (corrected > mag)) { // In Bounds + More than
        d_drift = -(((bound_num - derror) / bound_num) * max_drift);
    } else {
        frc::SmartDashboard::PutBoolean("Match", true);
    }

    double save_mag_c = corrected;
    corrected += (wraparound_to_change(last_short_sensor, short_sensor) + d_drift); //Find change in gyro and add drift
    corrected = range360(corrected);

    if (abs(mag - corrected) > corrected_bound) {
      if (mag > corrected) { //mag 0 mag_corrected 359
        d_drift = fabs(d_drift);
        save_mag_c += ((wraparound_to_change(last_short_sensor, short_sensor) - d_drift));
        save_mag_c = range360(save_mag_c);
        corrected = save_mag_c;

      } else if (mag < corrected) { //mag 360, other 0
        d_drift = fabs(d_drift);
        save_mag_c += ((wraparound_to_change(last_short_sensor, short_sensor) + d_drift));
        save_mag_c = range360(save_mag_c);
        corrected = save_mag_c;
      }

    }

    return corrected;
}

double SensorFusion::range360(double inp) {
    double out;
    if (inp < 0) {
        out = 360 + inp;
        out = fmod(out, 360);
    } else {
        out = fmod(inp, 360);
    }
    return out;
}

double SensorFusion::wraparound_to_change(double last, double cur) {
    double gyro_change = cur - last;
    if (fabs(gyro_change) > wrap_bound) {
        if (last > cur) {
            gyro_change = (360 - last) + cur;
        } else {
            gyro_change = (360 - cur) + last;
        }
    }
    return gyro_change;
}

void SensorFusion::encoders_to_coord(double left, double right) {
  double theta_new = -(right - left) / (dbaseline); //radians
  theta_new = fmod(theta_new * 180 / wpi::numbers::pi, 360); // degrees

/*
  double x = position[0];
  double y = position[1];
  double theta = position[2] * wpi::numbers::pi / 180; // theta should come in as a radian but output in degrees

  double dcenter = (left + right) / 2;
  double phi = (right - left) / dbaseline; // In radians

  double f_theta = theta + phi;
  f_theta = fmod(f_theta * 180 / wpi::numbers::pi, 360);
  double f_x = x + (dcenter * cos(theta));
  double f_y = y + (dcenter * sin(theta));
*/
  //position.at(0) = f_x;
  //position.at(1) = f_y;
  if (theta_new < 0) {
    enc_theta = fmod(theta_new + 360, 360);
  } else {
    enc_theta = fmod(theta_new, 360);
  }
}

double SensorFusion::get_magnometer(){
  double x_raw = gyro_imu->GetMagneticFieldX().value();
  double y_raw = gyro_imu->GetMagneticFieldY().value();

  double field_angle = atan2(y_raw, x_raw) * 180 / PI;

  
  if (field_angle < 0) {
    field_angle = 360 - (field_angle * -1);
  }

  double out = 360 - field_angle;
  return out;
}

double SensorFusion::get_gyro() {
  double raw_angle = gyro_imu->GetAngle().value();
  double new_angle;
  if (raw_angle < 0) {
    new_angle = 360 + raw_angle;
    return fmod(new_angle, 360);
  } else {
    return fmod(raw_angle, 360);
  }
}

void SensorFusion::gyro_drift() {
  double gyro = get_gyro(); // 0 - 360
  double mag = get_magnometer(); // 0 - 360
  double d_drift = 0;
  double derror = fabs(mag - mag_corrected); // error drift
  
  mag_corrected = driftAlgorithm(gyro, last_gyro, mag, derror, mag_corrected, bound_num, max_drift);
  last_gyro = gyro;
  
 
}

void SensorFusion::enc_drift() {
  double theta = enc_theta;
  double mag = get_magnometer();
  double d_drift = 0;
  double derror = fabs(mag - mag_corrected_enc);

  mag_corrected_enc = driftAlgorithm(enc_theta, last_enc, mag, derror, mag_corrected_enc, bound_num, max_drift);
  last_enc = enc_theta;
}

void SensorFusion::initializeSensors() {

    maginit = get_magnometer();
    gyro_imu->Reset();
    mag_corrected = maginit;
    mag_corrected_enc = maginit;
    last_gyro = 0.0;
    enc_theta = 0.0;
    last_enc = 0.0;
    
    lEncoder->SetPosition(0);
    rEncoder->SetPosition(0);
    lEncoder->SetPositionConversionFactor(1.96);
    rEncoder->SetPositionConversionFactor(1.96);
    position.push_back(0.0);
    position.push_back(0.0);
    position.push_back(0.0);
}

void SensorFusion::updateSensors(bool print) {
    encoders_to_coord(lEncoder->GetPosition(), rEncoder->GetPosition());
    gyro_drift();
    enc_drift();


    if (print) {
    //frc::SmartDashboard::PutNumber("Position X", position[0]);
    //frc::SmartDashboard::PutNumber("Position Y", position[1]);

    frc::SmartDashboard::PutNumber("Position theta", enc_theta);
    frc::SmartDashboard::PutNumber("Gyro", get_gyro());
    frc::SmartDashboard::PutNumber("Magnometer", get_magnometer());

    frc::SmartDashboard::PutNumber("Gyro Mag Corrected", mag_corrected);
    frc::SmartDashboard::PutNumber("Encoder Mag Corrected", mag_corrected_enc);
    }
}

double SensorFusion::getTrueGyro() {
    return range360(mag_corrected - maginit);
}

double SensorFusion::getTrueEnc() {
    return range360(mag_corrected_enc - maginit);
}