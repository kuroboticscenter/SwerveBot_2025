// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.DriveWithJoystick;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LEDSubsystem;

/** Add your docs here. */
public class RobotContainer {
  private final CommandXboxController m_controller = new CommandXboxController(0);
  Pigeon2 gyro = new Pigeon2(53, "rio");

  private final DriveBase m_swerve = new DriveBase(gyro::getRotation2d, new Pose2d());
  // private final SimDrivetrain m_simSwerve = new SimDrivetrain(new Pose2d());
  private final LEDSubsystem LEDSubsystem = new LEDSubsystem();

    public RobotContainer(double period) {
        m_swerve.setDefaultCommand(new DriveWithJoystick(m_swerve, m_controller, period));
    }
}
