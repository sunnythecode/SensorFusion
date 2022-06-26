// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_LL->RestoreFactoryDefaults();
  m_LF->RestoreFactoryDefaults();
  m_RL->RestoreFactoryDefaults();
  m_RF->RestoreFactoryDefaults();
  
  m_LL->SetInverted(true);
  m_LF->Follow(*m_LL, true);
  m_RL->SetInverted(false);
  m_RF->Follow(*m_RL, false);

  lEncoder.SetPosition(0);
  rEncoder.SetPosition(0);
  position.push_back(0.0);
  position.push_back(0.0);
  position.push_back(0.0);
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());

  encoders_to_coord(LLencoder_distance(), RLencoder_distance());


  frc::SmartDashboard::PutNumber("Position X", position[0]);
  frc::SmartDashboard::PutNumber("Position Y", position[1]);
  frc::SmartDashboard::PutNumber("Position theta", position[2]);
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  m_drive->ArcadeDrive(DzShift(controller->GetLeftY(), 0.2), DzShift(controller->GetRightX(), 0.2));
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
