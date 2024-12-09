// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(driveSubsystem.driveArcadeCommand(driverXbox::getLeftX, () -> {return driverXbox.getLeftY() * -1.0;}));
    driverXbox.leftBumper().onTrue(driveSubsystem.shiftHighCommand());
    driverXbox.rightBumper().onTrue(driveSubsystem.shiftLowCommand());

  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
