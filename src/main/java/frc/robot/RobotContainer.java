// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
  private final XboxController driverXbox = new XboxController(0);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private ElevatorSubsystem m_elevator = new ElevatorSubsystem();


  public RobotContainer() {
    driveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> driveSubsystem.driveArcade(driverXbox.getLeftY(),driverXbox.getLeftX()),
        driveSubsystem
      )
    );
    // driverXbox.leftBumper().onTrue(new InstantCommand(() -> driveSubsystem.shiftGearHighSpeed()));
    // driverXbox.rightBumper().onTrue(new InstantCommand(() -> driveSubsystem.shiftGearLowSpeed()));

     new JoystickButton(driverXbox, Button.kA.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L1));

    new JoystickButton(driverXbox, Button.kB.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L2));

    new JoystickButton(driverXbox, Button.kX.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L3));

    new JoystickButton(driverXbox, Button.kY.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L4));

  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
