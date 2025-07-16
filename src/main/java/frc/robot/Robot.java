// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class Robot extends TimedRobot {
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer(getPeriod());
    
  }

  @Override
  public void robotPeriodic() {
    // This runs in all robot modes (disabled, auto, teleop, test)
    CommandScheduler.getInstance().run();
    // m_swerve.periodic();
  }

  @Override
  public void autonomousInit () {
    m_robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {
    // m_swerve.updateOdometry();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }
}
