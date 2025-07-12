// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RangeFinder extends SubsystemBase {
  private final CANrange rangeFinder = new CANrange(50);

  /** Creates a new RangeFinder. */
  public RangeFinder() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public double getDistanceInMeters() {
    return rangeFinder.getDistance().getValueAsDouble();
  }
}
