// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <math.h> 
#pragma once
#define PI 3.14159265
#include <vector>
#include <string>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/DifferentialDrive.h>

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
  std::vector<double> encoders_to_coord(double left, double right, double x, double y, double theta);

  //robot baseline(width between right and left wheels in meters)
  double robotbaseline = 1.0;

  frc::XboxController* controller = new frc::XboxController{0};
  rev::CANSparkMax* m_RL = new rev::CANSparkMax(rMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_RF = new rev::CANSparkMax(rMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_LL = new rev::CANSparkMax(lMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_LF = new rev::CANSparkMax(lMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);
  
  frc::DifferentialDrive* m_drive = new frc::DifferentialDrive(*m_LL, *m_RL);
  rev::SparkMaxRelativeEncoder lEncoder = m_LL->GetEncoder();
  rev::SparkMaxRelativeEncoder rEncoder = m_RL->GetEncoder();





 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
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

std::vector<double> Robot::encoders_to_coord(double left, double right, double x, double y, double theta) {
  double dleft = left;
  double dright = dright;
  double dcenter = (dleft + dright) / 2;
  double phi = abs(dleft - dright) / 2;

  double f_theta = phi + theta;
  double f_x = x + (dcenter * cos(theta));
  double f_y = y + (dcenter * sin(theta));

  std::vector<double> arr = {f_x, f_y, f_theta};
  return arr;
}