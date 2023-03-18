// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import org.carlmontrobotics.robotcode2023.Constants.GoalPos;
import org.carlmontrobotics.robotcode2023.Constants.OI.Manipulator;
import org.carlmontrobotics.robotcode2023.Constants.Roller.RollerMode;
import org.carlmontrobotics.robotcode2023.commands.ArmPeriodic;
import org.carlmontrobotics.robotcode2023.commands.RunRoller;
import org.carlmontrobotics.robotcode2023.commands.SetArmWristPosition;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;
import org.carlmontrobotics.robotcode2023.subsystems.Roller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.CUBE;
import static org.carlmontrobotics.robotcode2023.Constants.Arm.CONE;

public class RobotContainer {

  public final Joystick driverController = new Joystick(0);
  public final Joystick manipulatorController = new Joystick(1);
  public final PowerDistribution pd = new PowerDistribution();

  public final Arm arm = new Arm();
  public final Roller roller = new Roller();

  public RobotContainer() {
    configureButtonBindingsDriver();
    configureButtonBindingsManipulator();
    arm.setDefaultCommand(new ArmPeriodic(
      arm, 
      () -> inputProcessing(getStickValue(manipulatorController, Axis.kLeftY)),
      () -> inputProcessing(getStickValue(manipulatorController, Axis.kRightX))
    ));
  }

  private void configureButtonBindingsDriver() {}

  private void configureButtonBindingsManipulator() {
    new JoystickButton(manipulatorController, Manipulator.toggleCubeButton)
      .onTrue(new InstantCommand(() -> {arm.object = CONE;}))
      .onFalse(new InstantCommand(() -> {arm.object = CUBE;}));
    new JoystickButton(manipulatorController, Manipulator.toggleFrontButton)
      .onTrue(new InstantCommand(() -> arm.isFront = false))
      .onFalse(new InstantCommand(() -> arm.isFront = true));
    new JoystickButton(manipulatorController, Manipulator.storeButton).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.STORED), arm.getWristGoal(GoalPos.STORED), arm)
    );
    new JoystickButton(manipulatorController, Manipulator.lowButton).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.LOW), arm.getWristGoal(GoalPos.LOW), arm)
    );
    new JoystickButton(manipulatorController, Manipulator.midButton).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.MID), arm.getWristGoal(GoalPos.MID), arm)
    );
    new JoystickButton(manipulatorController, Manipulator.highButton).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.HIGH), arm.getWristGoal(GoalPos.HIGH), arm)
    );
    new POVButton(driverController, 0).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.SHELF), arm.getWristGoal(GoalPos.SHELF), arm)
    );
    new POVButton(driverController, 90).onTrue(
      new InstantCommand(() -> {arm.object = CUBE;}).andThen(
        new SetArmWristPosition(arm.getArmGoal(GoalPos.INTAKE), arm.getWristGoal(GoalPos.INTAKE), arm)
      )
    );
    new POVButton(driverController, 180).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.SUBSTATION), arm.getWristGoal(GoalPos.SUBSTATION), arm)
    );
    new POVButton(driverController, 270).onTrue(
      new InstantCommand(() -> {arm.object = CONE;}).andThen(
        new SetArmWristPosition(arm.getArmGoal(GoalPos.INTAKE), arm.getWristGoal(GoalPos.INTAKE), arm)
      )
    );
    new JoystickButton(manipulatorController, Manipulator.rollerIntakeConeButton)
      .onTrue(new RunRoller(roller, RollerMode.INTAKE_CONE, Constants.Roller.conePickupColor));
    new JoystickButton(manipulatorController, Manipulator.rollerIntakeCubeButton)
      .onTrue(new RunRoller(roller, RollerMode.INTAKE_CUBE, Constants.Roller.conePickupColor));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
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
