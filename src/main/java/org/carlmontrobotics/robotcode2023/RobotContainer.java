// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import java.util.HashMap;

import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.lib199.path.PPRobotPath;
import org.carlmontrobotics.robotcode2023.Constants.OI.Driver;
import org.carlmontrobotics.robotcode2023.Constants.OI.Manipulator;
import org.carlmontrobotics.robotcode2023.commands.AlignChargingStation;
import org.carlmontrobotics.robotcode2023.commands.TeleopDrive;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  public final Joystick driverController = new Joystick(Driver.port);
  public final Joystick manipulatorController = new Joystick(Manipulator.port);
  public final PowerDistribution pd = new PowerDistribution();

  public final Limelight lime = new Limelight();
  public final Drivetrain drivetrain = new Drivetrain(lime);

  public final PPRobotPath[] autoPaths;
  public final DigitalInput[] autoSelectors;

  public RobotContainer() {

    autoPaths = new PPRobotPath[] {
      null,
      new PPRobotPath("New Path", drivetrain, false, new HashMap<>()),
      new PPRobotPath("3 game piece", drivetrain, false, new HashMap<>())
    };

    autoSelectors = new DigitalInput[Math.min(autoPaths.length, 26)];
    for(int i = 0; i < autoSelectors.length; i++) autoSelectors[i] = new DigitalInput(i);

    configureButtonBindingsDriver();
    configureButtonBindingsManipulator();

    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> inputProcessing(getStickValue(driverController, Axis.kLeftY)),
      () -> inputProcessing(getStickValue(driverController, Axis.kLeftX)),
      () -> inputProcessing(getStickValue(driverController, Axis.kRightX)),
      () -> driverController.getRawButton(Driver.slowDriveButton)
    ));
  }

  private void configureButtonBindingsDriver() {
    new JoystickButton(driverController, Driver.chargeStationAlignButton).onTrue(new AlignChargingStation(drivetrain));
    new JoystickButton(driverController, Driver.resetFieldOrientationButton).onTrue(new InstantCommand(drivetrain::resetFieldOrientation));
    new JoystickButton(driverController, Driver.toggleFieldOrientedButton).onTrue(new InstantCommand(() -> drivetrain.setFieldOriented(!drivetrain.getFieldOriented())));
  }
  private void configureButtonBindingsManipulator() {}

  public Command getAutonomousCommand() {
    // PPRobotPath autoPath = new PPRobotPath("New Path", drivetrain, false, new HashMap<>());
    PPRobotPath autoPath = null;
    for(int i = 0; i < autoSelectors.length; i++) {
      if(!autoSelectors[i].get()) {
        System.out.println("Using Path: " + i);
        autoPath = autoPaths[i];
        break;
      }
    }
    return autoPath == null ? new PrintCommand("No Autonomous Routine selected") : autoPath.getPathCommand(true, true);
    // return autoPath == null ? new PrintCommand("null :(") : autoPath.getPathCommand(true, true);
  }

  private double getStickValue(Joystick stick, Axis axis) {
    return stick.getRawAxis(axis.value) * (axis == Axis.kLeftY || axis == Axis.kRightY ? -1 : 1);
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
