// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include <wpi/numbers>

#include <frc/RobotController.h>

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  auto rightFeedforward = m_feedforward.Calculate(speeds.right);
  double leftOutput = m_leftPIDController.Calculate(m_leftEncoder.GetRate(),
                                                    speeds.left.value());
  double rightOutput = m_rightPIDController.Calculate(m_rightEncoder.GetRate(),
                                                      speeds.right.value());

  m_leftGroup.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightGroup.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot) {
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t(m_leftEncoder.GetDistance()),
                    units::meter_t(m_rightEncoder.GetDistance()));
}

void Drivetrain::ResetOdometry(const frc::Pose2d& pose) {
  m_leftEncoder.Reset();
  m_rightEncoder.Reset();
  m_drivetrainSimulator.SetPose(pose);
  m_odometry.ResetPosition(pose, m_gyro.GetRotation2d());
}

void Drivetrain::SimulationPeriodic() {
  // To update our simulation, we set motor voltage inputs, update the
  // simulation, and write the simulated positions and velocities to our
  // simulated encoder and gyro. We negate the right side so that positive
  // voltages make the right side move forward.
  m_drivetrainSimulator.SetInputs(units::volt_t{m_leftGroup.Get()} *
                                      frc::RobotController::GetInputVoltage(),
                                  units::volt_t{m_rightGroup.Get()} *
                                      frc::RobotController::GetInputVoltage());
  m_drivetrainSimulator.Update(20_ms);

  m_leftEncoderSim.SetDistance(m_drivetrainSimulator.GetLeftPosition().value());
  m_leftEncoderSim.SetRate(m_drivetrainSimulator.GetLeftVelocity().value());
  m_rightEncoderSim.SetDistance(
      m_drivetrainSimulator.GetRightPosition().value());
  m_rightEncoderSim.SetRate(m_drivetrainSimulator.GetRightVelocity().value());
  m_gyroSim.SetAngle(-m_drivetrainSimulator.GetHeading().Degrees().value());
}

void Drivetrain::Periodic() {
  UpdateOdometry();
  frc::SmartDashboard::PutNumber("idk", 2);
  m_fieldSim.SetRobotPose(m_odometry.GetPose());
  
  encoders_to_coord(m_leftEncoder.GetDistance(), m_rightEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Left", m_leftEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("right", m_rightEncoder.GetDistance());

  frc::SmartDashboard::PutNumber("Position X", position[0]);
  frc::SmartDashboard::PutNumber("Position Y", position[1]);
  frc::SmartDashboard::PutNumber("Position theta", position[2]);

}

void Drivetrain::encoders_to_coord(double left, double right) {
  double x = position[0];
  double y = position[1];
  double theta2 = position[2];
  double theta = fmod(theta2, 360) * wpi::numbers::pi / 180;
  double dleft = left;
  double dright = dright;
  double dcenter = (dleft + dright) / 2;
  double phi = abs(dleft - dright) / 2;

  double f_theta = phi + theta;
  double f_x = x + (dcenter * cos(theta));
  double f_y = y + (dcenter * sin(theta));

  position.at(0) = f_x;
  position.at(1) = f_y;
  f_theta = f_theta * 180 / wpi::numbers::pi;
  f_theta = fmod(f_theta, 360);
  position.at(2) = f_theta;

}