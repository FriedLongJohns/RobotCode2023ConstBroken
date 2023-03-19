// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import static org.carlmontrobotics.robotcode2023.Constants.OI.MIN_AXIS_TRIGGER_VALUE;

import java.util.function.BooleanSupplier;

import org.carlmontrobotics.robotcode2023.Constants.GoalPos;
import org.carlmontrobotics.robotcode2023.Constants.OI.Manipulator;
import org.carlmontrobotics.robotcode2023.Constants.Roller.RollerMode;
import org.carlmontrobotics.robotcode2023.commands.ArmTeleop;
import org.carlmontrobotics.robotcode2023.commands.RunRoller;
import org.carlmontrobotics.robotcode2023.commands.SetArmWristGoalPreset;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;
import org.carlmontrobotics.robotcode2023.subsystems.Roller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  public final Joystick driverController = new Joystick(0);
  public final Joystick manipulatorController = new Joystick(1);
  public final PowerDistribution pd = new PowerDistribution();

  public final Arm arm = new Arm();
  public final Roller roller = new Roller();

  public RobotContainer() {
    configureButtonBindingsDriver();
    configureButtonBindingsManipulator();
    arm.setDefaultCommand(new ArmTeleop(
      arm, 
      () -> inputProcessing(getStickValue(manipulatorController, Axis.kLeftY)),
      () -> inputProcessing(getStickValue(manipulatorController, Axis.kRightY))
    ));
  }

  private void configureButtonBindingsDriver() {}

  private void configureButtonBindingsManipulator() {
    BooleanSupplier isCube = () -> new JoystickButton(manipulatorController, Manipulator.toggleCubeButton).getAsBoolean();
    BooleanSupplier isFront = () -> new JoystickButton(manipulatorController, Manipulator.toggleFrontButton).getAsBoolean();

    new JoystickButton(manipulatorController, Manipulator.storePosButton).onTrue(new SetArmWristGoalPreset(GoalPos.STORED, isCube, isFront, arm));
    new JoystickButton(manipulatorController, Manipulator.lowPosButton).onTrue(new SetArmWristGoalPreset(GoalPos.LOW, isCube, isFront, arm));
    new JoystickButton(manipulatorController, Manipulator.midPosButton).onTrue(new SetArmWristGoalPreset(GoalPos.MID, isCube, isFront, arm));
    new JoystickButton(manipulatorController, Manipulator.highPosButton).onTrue(new SetArmWristGoalPreset(GoalPos.HIGH, isCube, isFront, arm));
    new POVButton(manipulatorController, Manipulator.shelfPickupPOV).onTrue(new SetArmWristGoalPreset(GoalPos.SHELF, isCube, isFront, arm));
    new POVButton(manipulatorController, Manipulator.intakeConePOV).onTrue(new SetArmWristGoalPreset(GoalPos.INTAKE, () -> false, isFront, arm));
    new POVButton(manipulatorController, Manipulator.substationPickupPOV).onTrue(new SetArmWristGoalPreset(GoalPos.STORED, isCube, isFront, arm));
    new POVButton(manipulatorController, Manipulator.intakeCubePOV).onTrue(new SetArmWristGoalPreset(GoalPos.STORED, () -> true, isFront, arm));

    axisTrigger(manipulatorController, Manipulator.rollerIntakeConeButton)
      .onTrue(new RunRoller(roller, RollerMode.INTAKE_CONE, Constants.Roller.conePickupColor));
    axisTrigger(manipulatorController, Manipulator.rollerIntakeCubeButton)
      .onTrue(new RunRoller(roller, RollerMode.INTAKE_CUBE, Constants.Roller.cubePickupColor));
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

  private Trigger axisTrigger(Joystick stick, Axis axis) {
    return new Trigger(() -> Math.abs(getStickValue(stick, axis)) > MIN_AXIS_TRIGGER_VALUE);
  }
}
