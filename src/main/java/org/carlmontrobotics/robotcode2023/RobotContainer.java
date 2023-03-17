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
    new JoystickButton(manipulatorController, Manipulator.toggleCubeCone).onTrue(new InstantCommand(() -> arm.toggleObjectType()));
    new JoystickButton(manipulatorController, Manipulator.toggleFrontBack).onTrue(new InstantCommand(() -> arm.toggleFront()));
    new JoystickButton(manipulatorController, Manipulator.store).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.STORED), arm.getWristGoal(GoalPos.STORED), arm)
    );
    new JoystickButton(manipulatorController, Manipulator.low).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.LOW), arm.getWristGoal(GoalPos.LOW), arm)
    );
    new JoystickButton(manipulatorController, Manipulator.mid).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.MID), arm.getWristGoal(GoalPos.MID), arm)
    );
    new JoystickButton(manipulatorController, Manipulator.high).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.HIGH), arm.getWristGoal(GoalPos.HIGH), arm)
    );
    new JoystickButton(manipulatorController, Manipulator.store).onTrue(
      new SetArmWristPosition(arm.getArmGoal(GoalPos.STORED), arm.getWristGoal(GoalPos.STORED), arm)
    );
    new JoystickButton(manipulatorController, Manipulator.rollerIntakeConeButton)
      .onTrue(new RunRoller(roller, RollerMode.INTAKE_CONE, Constants.Roller.conePickupColor));
    new JoystickButton(manipulatorController, Manipulator.rollerIntakeCubeButton)
      .onTrue(new RunRoller(roller, RollerMode.INTAKE_CUBE, Constants.Roller.cubePickupColor));
    new JoystickButton(manipulatorController, Manipulator.rollerOuttakeConeButton)
      .onFalse(new RunRoller(roller, RollerMode.OUTTAKE_CONE, Constants.Roller.conePickupColor));
    new JoystickButton(manipulatorController, Manipulator.rollerOuttakeCubeButton)
      .onFalse(new RunRoller(roller, RollerMode.OUTTAKE_CUBE, Constants.Roller.cubePickupColor));
    new JoystickButton(manipulatorController, Manipulator.rollerStopButton).onTrue(new InstantCommand(() -> roller.setSpeed(0)));
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
