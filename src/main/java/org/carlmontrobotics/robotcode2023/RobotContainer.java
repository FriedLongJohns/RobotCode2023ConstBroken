// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import org.carlmontrobotics.robotcode2023.commands.AlignChargingStation;
import org.carlmontrobotics.robotcode2023.commands.TeleopDrive;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  public final Joystick driverController = new Joystick(0);
  public final Joystick manipulatorController = new Joystick(1);
  public final PowerDistribution pd = new PowerDistribution();

  public final Drivetrain drivetrain = new Drivetrain();

  public RobotContainer() {
    configureButtonBindingsDriver();
    configureButtonBindingsManipulator();

    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> inputProcessing(getStickValue(driverController, Axis.kLeftY)),
      () -> inputProcessing(getStickValue(driverController, Axis.kLeftX)),
      () -> inputProcessing(getStickValue(driverController, Axis.kRightX)),
      () -> driverController.getRawButton(Constants.OI.Driver.slowDriveButton)
    ));
  }

  private void configureButtonBindingsDriver() {
    new JoystickButton(driverController, XboxController.Button.kA.value).onTrue(new AlignChargingStation(drivetrain));
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(drivetrain::resetFieldOrientation));
  }
  private void configureButtonBindingsManipulator() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private double getStickValue(Joystick stick, XboxController.Axis axis) {
    return stick.getRawAxis(axis.value) * (axis == XboxController.Axis.kLeftY || axis == XboxController.Axis.kRightY ? -1 : 1);
  }

  /**
   * Processes an input from the joystick into a value between -1 and 1
   * 
   * @param value The value to be processed.
   * @return The processed value.
   */
  private double inputProcessing(double value) {
    double processedInput;
    // processedInput =
    // (((1-Math.cos(value*Math.PI))/2)*((1-Math.cos(value*Math.PI))/2))*(value/Math.abs(value));
    processedInput = Math.copySign(((1 - Math.cos(value * Math.PI)) / 2) * ((1 - Math.cos(value * Math.PI)) / 2),
        value);
    return processedInput;
  }

}
