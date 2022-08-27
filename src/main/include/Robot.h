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
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/ADIS16448_IMU.h>

class Robot : public frc::TimedRobot {
 public:
  //Fix these:
  int rMotorLeaderID = 15;
  int rMotorFollowerID = 14;
  int lMotorLeaderID = 12;
  int lMotorFollowerID = 13;
  
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
  double LLencoder_distance();
  double RLencoder_distance();
  double get_magnometer();
  
  std::vector<double> position;
  frc::ADIS16448_IMU* gyro_imu = new frc::ADIS16448_IMU;
  //robot baseline(width between right and left wheels in meters)
  double dbaseline = 1.0;

  frc::XboxController* controller = new frc::XboxController{0};
  rev::CANSparkMax* m_RL = new rev::CANSparkMax(rMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_RF = new rev::CANSparkMax(rMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_LL = new rev::CANSparkMax(lMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_LF = new rev::CANSparkMax(lMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);
  
  frc::DifferentialDrive* m_drive = new frc::DifferentialDrive(*m_LL, *m_RL);
  rev::SparkMaxRelativeEncoder lEncoder = m_LL->GetEncoder();
  rev::SparkMaxRelativeEncoder rEncoder = m_RL->GetEncoder();





 private:
};


double Robot::DzShift(double input, double dz) {
    double speed;
    if (fabs(input) < dz) {
        return 0.0;
    }
    if (input < 0) {
        double m = (1/(1-dz));
        double out = (m*(input-1))+1;
        return (out * out);
    } else {
        double m = (1/(1-dz));
        double out = (m*(fabs(input)-1))+1;
        return -(out * out);
    }
}


void Robot::encoders_to_coord(double left, double right) {
  double x = position[0];
  double y = position[1];
  double theta = position[2] * wpi::numbers::pi / 180; // theta should come in as a radian but output in degrees

  double dcenter = (left + right) / 2;
  double phi = (right - left) / dbaseline; // In radians

  double f_theta = theta + phi;
  f_theta = fmod(f_theta * 180 / wpi::numbers::pi, 360);
  double f_x = x + (dcenter * cos(theta));
  double f_y = y + (dcenter * sin(theta));

  position.at(0) = f_x;
  position.at(1) = f_y;
  position.at(2) = f_theta;

}




double Robot::LLencoder_distance(){
  lEncoder.SetPositionConversionFactor(1);
  //return (lEncoder.GetPosition() / 12 * (PI * 4));
  return ((lEncoder.GetPosition() / 4.15) * (PI * 4) / 12); // 4.15 is ticks per rotation
}

double Robot::RLencoder_distance(){
  rEncoder.SetPositionConversionFactor(1);
  //return (lEncoder.GetPosition() / 12 * (PI * 4));
  return ((lEncoder.GetPosition() / 4.15) * (PI * 4) / 12); // 4.15 is ticks per rotation
}

double Robot::get_magnometer(){
  double x_raw = gyro_imu->GetMagneticFieldX().value();
  double y_raw = gyro_imu->GetMagneticFieldY().value();

  double field_angle = atan(x_raw/y_raw);

  return field_angle;
}
