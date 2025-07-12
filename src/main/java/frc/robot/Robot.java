// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.DriveWithJoystick;

import com.ctre.phoenix6.hardware.Pigeon2;

public class Robot extends TimedRobot {
  private final CommandXboxController m_controller = new CommandXboxController(0);
  Pigeon2 gyro = new Pigeon2(53, "rio");
  
  private final Drivetrain m_swerve = new Drivetrain(gyro::getRotation2d, new Pose2d());
  // private final SimDrivetrain m_simSwerve = new SimDrivetrain(new Pose2d());

public Robot() {
  m_swerve.setDefaultCommand(new DriveWithJoystick(m_swerve, m_controller, getPeriod()));
}

  @Override
  public void robotPeriodic() {
      // This runs in all robot modes (disabled, auto, teleop, test)
      CommandScheduler.getInstance().run();
      // m_swerve.periodic();
  }

  @Override
  public void autonomousPeriodic() {
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    // new DriveWithJoystick(m_swerve, m_controller, getPeriod());
    // driveWithJoystick(true);
  }
}
