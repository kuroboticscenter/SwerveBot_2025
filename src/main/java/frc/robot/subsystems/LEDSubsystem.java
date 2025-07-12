// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSubsystem extends SubsystemBase {
  CANdle LED1 = new CANdle(51, "rio");
  CANdle LED2 = new CANdle(52, "rio");
  
  // Colors
  RGBWColor rslOrange = new RGBWColor(new Color(200, 25, 0));
  RGBWColor white = new RGBWColor(Color.kWhite);
  RGBWColor black = new RGBWColor(Color.kBlack);
  
  // Controls
  SolidColor rslDisabled = new SolidColor(0, 7).withColor(rslOrange);
  StrobeAnimation rslEnabled = new StrobeAnimation(0, 7)
    .withColor(rslOrange)
    .withFrameRate(1.5);
    
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    LEDConfigs ledConfigs = new LEDConfigs().withBrightnessScalar(.1);

    LED1.getConfigurator().apply(ledConfigs);
    LED2.getConfigurator().apply(ledConfigs);
  
    LED1.setControl(new SolidColor(0, 7).withColor(black));
    LED2.setControl(new SolidColor(0, 7).withColor(black));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
