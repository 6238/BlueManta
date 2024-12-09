// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> driveSubsystem.driveArcade(driverXbox.getLeftY(),driverXbox.getLeftX()),
        driveSubsystem
      )
    );
    driverXbox.leftBumper().onTrue(new InstantCommand(() -> driveSubsystem.shiftGearHighSpeed()));
    driverXbox.rightBumper().onTrue(new InstantCommand(() -> driveSubsystem.shiftGearLowSpeed()));
  
  }
  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
