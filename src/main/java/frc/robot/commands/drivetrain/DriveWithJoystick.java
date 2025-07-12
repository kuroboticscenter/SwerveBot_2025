// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.SwerveConstants;
import frc.robot.subsystems.DriveBase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithJoystick extends Command {
  CommandXboxController driveController;
  DriveBase drivetrain;
  double period;
  
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(20);
  
  /** Creates a new TeleopDrive. */
  public DriveWithJoystick(DriveBase drivetrainSubSystem, CommandXboxController controller, double period) {
    driveController = controller;
    drivetrain = drivetrainSubSystem;
    this.period = period;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(driveController.getLeftY(), 0.05))
          * SwerveConstants.TOP_SPEED_METERS_PER_SEC;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
      -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driveController.getLeftX(), 0.05))
        * SwerveConstants.TOP_SPEED_METERS_PER_SEC;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = 
      -m_rotLimiter.calculate(MathUtil.applyDeadband(driveController.getRightX(), 0.05))
        * (SwerveConstants.TOP_SPEED_METERS_PER_SEC / 2);

    drivetrain.drive(xSpeed, ySpeed, rot, true, 0.02);
    // m_simSwerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
