// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <math.h> 
#pragma once
#define PI 3.14159265
#include <vector>
#include <string>

#include <wpi/numbers>
#include <frc/Joystick.h>
#include "SFDrive.h"
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/ADIS16448_IMU.h>

class Robot : public frc::TimedRobot {
 public:
  //Fix these:
  int rMotorLeaderID = 9;
  int rMotorFollowerID = 3;
  int lMotorLeaderID = 15;
  int lMotorFollowerID = 16;
  

  //Functions
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  double DzShift(double input, double dz);
  //Function prone to skidding error and stuff(not optimized)
  void encoders_to_coord(double left, double right);
  double get_magnometer();
  double get_gyro();
  void gyro_edit();


  //Sensor Drift
  double bound_num;
  double maginit;
  double max_drift = 0.4;

  double gyro_corrected;
  double last_gyro_corrected = 0.0;

  double last_gyro = 0.0;
  //robot baseline(width between right and left wheels inches)
  double dbaseline = 26.0;


  //Postion
  std::vector<double> position;


  //Gyro Stuff
  frc::ADIS16448_IMU* gyro_imu = new frc::ADIS16448_IMU;
  

  //Controller
  frc::XboxController* ctr = new frc::XboxController{0};

  //Motors
  rev::CANSparkMax* m_RL = new rev::CANSparkMax(rMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_RF = new rev::CANSparkMax(rMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_LL = new rev::CANSparkMax(lMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_LF = new rev::CANSparkMax(lMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);

  //Drive Base:
  SFDrive* m_drive = new SFDrive(m_LL, m_RL);

  
  //Encoders
  rev::SparkMaxRelativeEncoder lEncoder = m_LL->GetEncoder();
  rev::SparkMaxRelativeEncoder rEncoder = m_RL->GetEncoder();





 private:
};


void Robot::encoders_to_coord(double left, double right) {
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







double Robot::get_magnometer(){
  double x_raw = gyro_imu->GetMagneticFieldX().value();
  double y_raw = gyro_imu->GetMagneticFieldY().value();

  double field_angle = atan2(y_raw, x_raw) * 180 / PI;

  
  if (field_angle < 0) {
    field_angle = 360 - (field_angle * -1);
  }

  return (360 - field_angle);
}

double Robot::get_gyro() {
  double raw_angle = gyro_imu->GetAngle().value();
  double new_angle;
  if (raw_angle < 0) {
    new_angle = 360 + raw_angle;
    return fmod(new_angle, 360);
  } else {
    return fmod(raw_angle, 360);
  }
}

void Robot::gyro_edit() {
  double gyro = get_gyro();
  double mag = get_magnometer();
  double d_drift = 0;
  double derror = fabs(mag - gyro_corrected);
  //frc::SmartDashboard::PutNumber("Derror", derror);
  
  if ((derror >= bound_num) && (gyro_corrected < mag)) {
    d_drift = max_drift;
    frc::SmartDashboard::PutNumber("Error Line 156 Robot.h", 0);
  } else if ((derror < bound_num) && (gyro_corrected < mag)) {
    d_drift = ((bound_num - derror) / bound_num) * max_drift;
    frc::SmartDashboard::PutNumber("Error Line 156 Robot.h", 0);
  } if ((derror >= bound_num) && (gyro_corrected > mag)) {
    d_drift = -max_drift;
    frc::SmartDashboard::PutNumber("Error Line 156 Robot.h", 0);
  } else if ((derror < bound_num) && (gyro_corrected > mag)) {
    d_drift = -(((bound_num - derror) / bound_num) * max_drift);
    frc::SmartDashboard::PutNumber("Error Line 156 Robot.h", 0);
  }
    
  else {
    frc::SmartDashboard::PutNumber("Error Line 156 Robot.h", -1);
  }
  //Case 1: error outside bound and less


  gyro_corrected += ((gyro - last_gyro) + d_drift); //Find change in gyro and add drift
  last_gyro = gyro;
  //last_gyro_corrected = gyro_corrected;
  
 
}



