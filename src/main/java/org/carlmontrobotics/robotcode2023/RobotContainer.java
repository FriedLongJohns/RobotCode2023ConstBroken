// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import org.carlmontrobotics.robotcode2023.subsystems.Arm;
import org.carlmontrobotics.robotcode2023.subsystems.Wrist;

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
  public final Wrist wrist = new Wrist();

  public RobotContainer() {
    configureButtonBindingsDriver();
    configureButtonBindingsManipulator();
  }

  private void configureButtonBindingsDriver() {}
  private void configureButtonBindingsManipulator() {
      new JoystickButton(manipulatorController, Constants.OI.Controller.Y).onTrue(new InstantCommand(() -> {
        arm.setGoalPos.accept(Arm.ArmPreset.HIGH.value);
        wrist.setGoalPos.accept(Wrist.WristPreset.HIGH.value);
      }));
      new JoystickButton(manipulatorController, Constants.OI.Controller.B).onTrue(new InstantCommand(() -> {
        arm.setGoalPos.accept(Arm.ArmPreset.MID.value);
        wrist.setGoalPos.accept(Wrist.WristPreset.MID.value);
      }));
      new JoystickButton(manipulatorController, Constants.OI.Controller.A).onTrue(new InstantCommand(() -> {
        arm.setGoalPos.accept(Arm.ArmPreset.INTAKE.value);
        wrist.setGoalPos.accept(Wrist.WristPreset.INTAKE.value);
      }));
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
