// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  CANdle led = new CANdle(40);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.5; // dim the LEDs to half brightness
    led.configAllSettings(config);
    this.setDefaultCommand(this.rainbowCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command rainbowCommand() {
    return runOnce(() -> {
      RainbowAnimation anim = new RainbowAnimation(0.5, 0.8, 308);
      led.animate(anim);
    }).ignoringDisable(true);
  }
}
