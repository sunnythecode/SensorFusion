// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "SensorFusion.h"
#pragma once
#define PI 3.14159265

#include <vector>
#include <string>
#include <math.h> 
#include <wpi/numbers>


#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/ADIS16448_IMU.h>



SensorFusion::SensorFusion(rev::SparkMaxRelativeEncoder* lEncoder, rev::SparkMaxRelativeEncoder* rEncoder): lEncoder{lEncoder}, rEncoder{rEncoder} {}


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
  position.at(2) = theta_new;
}

double SensorFusion::get_magnometer(){
  double x_raw = gyro_imu->GetMagneticFieldX().value();
  double y_raw = gyro_imu->GetMagneticFieldY().value();

  double field_angle = atan2(y_raw, x_raw) * 180 / PI;

  
  if (field_angle < 0) {
    field_angle = 360 - (field_angle * -1);
  }

  return (360 - field_angle);
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
  double gyro = get_gyro();
  double mag = get_magnometer();
  double d_drift = 0;
  double derror = fabs(mag - mag_corrected);
  
  if ((derror >= bound_num) && (mag_corrected < mag)) { // Out of bounds + Less than
    d_drift = max_drift;
    frc::SmartDashboard::PutNumber("Error Line 156 Robot.h", 0);
  } else if ((derror < bound_num) && (mag_corrected < mag)) { // In Bounds + Less than
    d_drift = ((bound_num - derror) / bound_num) * max_drift;
    frc::SmartDashboard::PutNumber("Error Line 156 Robot.h", 0);
  } if ((derror >= bound_num) && (mag_corrected > mag)) { // Out of Bounds + More than
    d_drift = -max_drift;
    frc::SmartDashboard::PutNumber("Error Line 156 Robot.h", 0);
  } else if ((derror < bound_num) && (mag_corrected > mag)) { // In Bounds + More than
    d_drift = -(((bound_num - derror) / bound_num) * max_drift);
    frc::SmartDashboard::PutNumber("Error Line 156 Robot.h", 0);
  }
    
  else {
    frc::SmartDashboard::PutNumber("Error Line 156 Robot.h", -1);
  }


  mag_corrected += ((gyro - last_gyro) + d_drift); //Find change in gyro and add drift
  last_gyro = gyro;
  
 
}

void SensorFusion::initializeSensors() {

    maginit = get_magnometer();
    gyro_imu->Reset();
    mag_corrected = 0.0;

    lEncoder->SetPosition(0);
    rEncoder->SetPosition(0);
    lEncoder->SetPositionConversionFactor(1.96);
    rEncoder->SetPositionConversionFactor(1.96);
    position.push_back(0.0);
    position.push_back(0.0);
    position.push_back(0.0);
}

void SensorFusion::updateSensors() {
    frc::SmartDashboard::PutNumber("lEncoder", lEncoder->GetPosition());
    frc::SmartDashboard::PutNumber("rEncoder", rEncoder->GetPosition());

    encoders_to_coord(lEncoder->GetPosition(), rEncoder->GetPosition());
    gyro_drift();



    //frc::SmartDashboard::PutNumber("Position X", position[0]);
    //frc::SmartDashboard::PutNumber("Position Y", position[1]);
    frc::SmartDashboard::PutNumber("Position theta", position[2]);
    frc::SmartDashboard::PutNumber("Magnometer", get_magnometer());

    frc::SmartDashboard::PutNumber("Gyro", get_gyro());
    frc::SmartDashboard::PutNumber("Gyro Corrected", mag_corrected);

}

double SensorFusion::Get_trueGyro() {
    return (mag_corrected - maginit);
}