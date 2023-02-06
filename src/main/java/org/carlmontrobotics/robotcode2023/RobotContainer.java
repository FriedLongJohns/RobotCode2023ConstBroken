// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import org.carlmontrobotics.robotcode2023.subsystems.Arm;
import org.carlmontrobotics.robotcode2023.subsystems.Arm.ArmPreset;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  public final Joystick driverController = new Joystick(0);
  public final Joystick manipulatorController = new Joystick(1);
  public final PowerDistribution pd = new PowerDistribution();
  public final Arm arm = new Arm();

  public RobotContainer() {
    configureButtonBindingsDriver();
    configureButtonBindingsManipulator();
  }

  private void configureButtonBindingsDriver() {}
  private void configureButtonBindingsManipulator() {
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.cone).and(new JoystickButton(manipulatorController, Constants.OI.Manipulator.intake)).whileTrue(new InstantCommand(()->arm.setPreset(ArmPreset.CONEINTAKE)));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.cone).and(new JoystickButton(manipulatorController, Constants.OI.Manipulator.outtakeLow)).whileTrue(new InstantCommand(()->arm.setPreset(ArmPreset.CONELOWROW)));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.cone).and(new JoystickButton(manipulatorController, Constants.OI.Manipulator.outtakeMid)).whileTrue(new InstantCommand(()->arm.setPreset(ArmPreset.CONEMIDROW)));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.cone).and(new JoystickButton(manipulatorController, Constants.OI.Manipulator.outtakeHigh)).whileTrue(new InstantCommand(()->arm.setPreset(ArmPreset.CONEHIGHROW)));

    new JoystickButton(manipulatorController, Constants.OI.Manipulator.cube).and(new JoystickButton(manipulatorController, Constants.OI.Manipulator.intake)).whileTrue(new InstantCommand(()->arm.setPreset(ArmPreset.CUBEINTAKE)));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.cube).and(new JoystickButton(manipulatorController, Constants.OI.Manipulator.outtakeLow)).whileTrue(new InstantCommand(()->arm.setPreset(ArmPreset.CUBELOWROW)));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.cube).and(new JoystickButton(manipulatorController, Constants.OI.Manipulator.outtakeMid)).whileTrue(new InstantCommand(()->arm.setPreset(ArmPreset.CUBEMIDROW)));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.cube).and(new JoystickButton(manipulatorController, Constants.OI.Manipulator.outtakeHigh)).whileTrue(new InstantCommand(()->arm.setPreset(ArmPreset.CUBEHIGHROW)));
    /*
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.cycleUp).whileTrue(new InstantCommand(()->arm.cycleUp()));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.cycleDown).whileTrue(new InstantCommand(()->arm.cycleDown()));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.swapItemType).whileTrue(new InstantCommand(()->arm.swapType()));
    */
  }

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
