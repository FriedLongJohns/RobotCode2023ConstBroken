// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import org.carlmontrobotics.robotcode2023.Constants.GoalPos;
import org.carlmontrobotics.robotcode2023.Constants.OI.Controller;
import org.carlmontrobotics.robotcode2023.commands.ArmPeriodic;
import org.carlmontrobotics.robotcode2023.commands.SetArmWristPosition;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Axis;

public class RobotContainer {

  public final Joystick driverController = new Joystick(0);
  public final Joystick manipulatorController = new Joystick(1);
  public final PowerDistribution pd = new PowerDistribution();
  public final Arm arm = new Arm();

  public RobotContainer() {
    configureButtonBindingsDriver();
    configureButtonBindingsManipulator();
    arm.setDefaultCommand(new ArmPeriodic(
      arm, 
      () -> inputProcessing(getStickValue(manipulatorController, Axis.kLeftY)),
      () -> inputProcessing(getStickValue(manipulatorController, Axis.kRightX))
    ));
  }
  //need to update the buttons
  private void configureButtonBindingsDriver() {}
  private void configureButtonBindingsManipulator() {
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.toggleCubeCone).onTrue(new InstantCommand(() -> arm.toggleCube()));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.toggleFrontBack).onTrue(new InstantCommand(() -> arm.toggleFront()));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.store).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.STORED), arm.getWristGoal(GoalPos.STORED), false, arm)
    );
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.low).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.LOW), arm.getWristGoal(GoalPos.LOW), true, arm)
    );
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.mid).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.MID), arm.getWristGoal(GoalPos.MID), true, arm)
    );
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.high).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.HIGH), arm.getWristGoal(GoalPos.HIGH), true, arm)
    );
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.store).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.STORED), arm.getWristGoal(GoalPos.STORED), true, arm)
    );
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
