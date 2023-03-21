// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import static org.carlmontrobotics.robotcode2023.Constants.OI.MIN_AXIS_TRIGGER_VALUE;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.lib199.path.PPRobotPath;
import org.carlmontrobotics.robotcode2023.Constants.GoalPos;
import org.carlmontrobotics.robotcode2023.Constants.OI.Driver;
import org.carlmontrobotics.robotcode2023.Constants.OI.Manipulator;
import org.carlmontrobotics.robotcode2023.Constants.Roller.RollerMode;
import org.carlmontrobotics.robotcode2023.commands.AlignChargingStation;
import org.carlmontrobotics.robotcode2023.commands.ArmTeleop;
import org.carlmontrobotics.robotcode2023.commands.RotateToFieldRelativeAngle;
import org.carlmontrobotics.robotcode2023.commands.RunRoller;
import org.carlmontrobotics.robotcode2023.commands.SetArmWristGoalPreset;
import org.carlmontrobotics.robotcode2023.commands.TeleopDrive;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;
import org.carlmontrobotics.robotcode2023.subsystems.Roller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  public final Joystick driverController = new Joystick(Driver.port);
  public final Joystick manipulatorController = new Joystick(Manipulator.port);

  public final PowerDistribution pd = new PowerDistribution();

  public final Limelight lime = new Limelight();
  public final Drivetrain drivetrain = new Drivetrain(lime);
  public final Arm arm = new Arm();
  public final Roller roller = new Roller(drivetrain);

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

    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> inputProcessing(getStickValue(driverController, Axis.kLeftY)),
      () -> inputProcessing(getStickValue(driverController, Axis.kLeftX)),
      () -> inputProcessing(getStickValue(driverController, Axis.kRightX)),
      () -> driverController.getRawButton(Driver.slowDriveButton)
    ));

    configureButtonBindingsDriver();
    configureButtonBindingsManipulator();
    arm.setDefaultCommand(new ArmTeleop(
      arm,
      () -> inputProcessing(getStickValue(manipulatorController, Axis.kLeftY)),
      () -> inputProcessing(getStickValue(manipulatorController, Axis.kRightY))
    ));
  }

  private void configureButtonBindingsDriver() {
    new JoystickButton(driverController, Driver.chargeStationAlignButton).onTrue(new AlignChargingStation(drivetrain));
    new JoystickButton(driverController, Driver.resetFieldOrientationButton).onTrue(new InstantCommand(drivetrain::resetFieldOrientation));
    new JoystickButton(driverController, Driver.toggleFieldOrientedButton).onTrue(new InstantCommand(() -> drivetrain.setFieldOriented(!drivetrain.getFieldOriented())));
    axisTrigger(driverController, Driver.driveToPointButton).onTrue(new InstantCommand(() -> drivetrain.testDriveToPoint()));

    new JoystickButton(driverController, Driver.rotateToFieldRelativeAngle0Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(0), drivetrain));
    new JoystickButton(driverController, Driver.rotateToFieldRelativeAngle90Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(-90), drivetrain));
    new JoystickButton(driverController, Driver.rotateToFieldRelativeAngle180Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(180), drivetrain));
    new JoystickButton(driverController, Driver.rotateToFieldRelativeAngle270Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));
  }

  private void configureButtonBindingsManipulator() {
    BooleanSupplier isCube = () -> new JoystickButton(manipulatorController, Manipulator.toggleCubeButton).getAsBoolean();
    BooleanSupplier isFront = () -> new JoystickButton(manipulatorController, Manipulator.toggleFrontButton).getAsBoolean();
    //BooleanSupplier isStopped = () -> new JoystickButton(manipulatorController, Manipulator.stopRollerButton).getAsBoolean();

    new JoystickButton(manipulatorController, Manipulator.storePosButton).onTrue(new SetArmWristGoalPreset(GoalPos.STORED, isCube, isFront, arm));
    new JoystickButton(manipulatorController, Manipulator.lowPosButton).onTrue(new SetArmWristGoalPreset(GoalPos.LOW, isCube, isFront, arm));
    new JoystickButton(manipulatorController, Manipulator.midPosButton).onTrue(new SetArmWristGoalPreset(GoalPos.MID, isCube, isFront, arm));
    new JoystickButton(manipulatorController, Manipulator.highPosButton).onTrue(new SetArmWristGoalPreset(GoalPos.HIGH, isCube, isFront, arm));
    new POVButton(manipulatorController, Manipulator.shelfPickupPOV).onTrue(new SetArmWristGoalPreset(GoalPos.SHELF, isCube, isFront, arm));
    new POVButton(manipulatorController, Manipulator.intakeConePOV).onTrue(new SetArmWristGoalPreset(GoalPos.INTAKE, () -> false, isFront, arm));
    new POVButton(manipulatorController, Manipulator.substationPickupPOV).onTrue(new SetArmWristGoalPreset(GoalPos.STORED, isCube, isFront, arm));
    new POVButton(manipulatorController, Manipulator.intakeCubePOV).onTrue(new SetArmWristGoalPreset(GoalPos.STORED, () -> true, isFront, arm));
    // axisTrigger(manipulatorController, Manipulator.rollerIntakeConeButton)
    //   .onTrue(new RunRoller(roller, RollerMode.INTAKE_CONE, Constants.Roller.conePickupColor));
    axisTrigger(manipulatorController, Manipulator.rollerIntakeCubeButton)
      .onTrue(new ConditionalCommand(
        new RunRoller(roller, RollerMode.INTAKE_CUBE, Constants.Roller.cubePickupColor), 
        new RunRoller(roller, RollerMode.OUTTAKE_CUBE, Constants.Roller.defaultColor), 
        isCube
      ));
    axisTrigger(manipulatorController, Manipulator.rollerIntakeConeButton)
      .onTrue(new ConditionalCommand(
        new RunRoller(roller, RollerMode.INTAKE_CONE, Constants.Roller.conePickupColor), 
        new RunRoller(roller, RollerMode.OUTTAKE_CONE, Constants.Roller.defaultColor), 
        isCube
      ));
      new JoystickButton(manipulatorController, Manipulator.stopRollerButton).onTrue(new InstantCommand(() -> roller.setSpeed(0), roller));
  }

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

  /**
   * Returns a new instance of Trigger based on the given Joystick and Axis objects.
   * The Trigger is triggered when the absolute value of the stick value on the specified axis
   * exceeds a minimum threshold value.
   * 
   * @param stick The Joystick object to retrieve stick value from.
   * @param axis The Axis object to retrieve value from the Joystick.
   * @return A new instance of Trigger based on the given Joystick and Axis objects.
   * @throws NullPointerException if either stick or axis is null.
   */
  private Trigger axisTrigger(Joystick stick, Axis axis) {
    return new Trigger(() -> Math.abs(getStickValue(stick, axis)) > MIN_AXIS_TRIGGER_VALUE);
  }
}
