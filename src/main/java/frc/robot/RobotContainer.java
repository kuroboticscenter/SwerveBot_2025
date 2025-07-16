// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drivetrain.DriveWithJoystick;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RangeFinder;

/** Add your docs here. */
public class RobotContainer {
  private final CommandXboxController m_controller = new CommandXboxController(0);
  Pigeon2 gyro = new Pigeon2(53, "rio");

  private final DriveBase m_swerve = new DriveBase(gyro::getRotation2d, new Pose2d());
  // private final SimDrivetrain m_simSwerve = new SimDrivetrain(new Pose2d());
  private final LEDSubsystem LEDSubsystem = new LEDSubsystem();
  private final RangeFinder RangeFinderSubsystem = new RangeFinder();

  private Trigger isEnabledTrigger = new Trigger(DriverStation::isEnabled);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer(double period) {
    m_swerve.setDefaultCommand(new DriveWithJoystick(m_swerve, m_controller, period));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    isEnabledTrigger.onTrue(LEDSubsystem.setLEDEnabled()).onFalse(LEDSubsystem.setLEDDisabled());

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
