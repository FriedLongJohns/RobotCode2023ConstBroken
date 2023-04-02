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
import org.carlmontrobotics.robotcode2023.commands.HoldRoller;
import org.carlmontrobotics.robotcode2023.commands.RotateToFieldRelativeAngle;
import org.carlmontrobotics.robotcode2023.commands.RunRoller;
import org.carlmontrobotics.robotcode2023.commands.SetArmWristGoalPreset;
import org.carlmontrobotics.robotcode2023.commands.SetArmWristPositionV3;
import org.carlmontrobotics.robotcode2023.commands.TeleopDrive;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;
import org.carlmontrobotics.robotcode2023.subsystems.Roller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  public final Joystick driverController = new Joystick(Driver.port);
  public final Joystick manipulatorController = new Joystick(Manipulator.port);

  public final PowerDistribution pd = new PowerDistribution();

  public final Limelight lime = new Limelight();
  public final Drivetrain drivetrain = new Drivetrain();
  public final Arm arm = new Arm();
  public final Roller roller = new Roller();

  public final PPRobotPath[] autoPaths;
  public final DigitalInput[] autoSelectors;
  public HashMap<String, Command> eventMap;

  public RobotContainer() {

    eventMap = new HashMap<>();

    {
      eventMap.put("Cone High Pos.", new SetArmWristGoalPreset(GoalPos.HIGH, () -> false, () -> false, arm));
      eventMap.put("Stored Pos.", new SetArmWristGoalPreset(GoalPos.STORED, () -> false, () -> false, arm));
      eventMap.put("Run Cube Intake", new SequentialCommandGroup(new SetArmWristGoalPreset(GoalPos.INTAKE, () -> true, () -> false, arm), new RunRoller(roller, RollerMode.INTAKE_CUBE, Constants.Roller.cubePickupColor)));
      eventMap.put("Cube High Pos.", new SetArmWristGoalPreset(GoalPos.HIGH, () -> true, () -> false, arm));
      eventMap.put("Run Cube Outtake", new RunRoller(roller, RollerMode.OUTTAKE_CUBE, Constants.Roller.defaultColor));
      eventMap.put("Run Cone Intake", new SequentialCommandGroup(new SetArmWristGoalPreset(GoalPos.INTAKE, () -> false, () -> false, arm), new RunRoller(roller, RollerMode.INTAKE_CONE, Constants.Roller.conePickupColor)));
      eventMap.put("Run Cone Outtake", new RunRoller(roller, RollerMode.OUTTAKE_CONE, Constants.Roller.defaultColor));
      eventMap.put("Move Arm Back", new SetArmWristPositionV3((-5*Math.PI)/8, Constants.Arm.WRIST_STOW_POS_RAD, arm));
      eventMap.put("Cone Intake Pos.", new SetArmWristGoalPreset(GoalPos.INTAKE, () -> false, () -> false, arm));
      eventMap.put("Cube Intake Pos.", new SetArmWristGoalPreset(GoalPos.INTAKE, () -> true, () -> false, arm));
      eventMap.put("Field Rotate 90", new RotateToFieldRelativeAngle(new Rotation2d(90), drivetrain));
      eventMap.put("Stop", stopDt());
      eventMap.put("Auto-Align", new ProxyCommand(() -> new AlignChargingStation(drivetrain)));
      eventMap.put("PrintAlign", new PrintCommand("Aligning"));
      eventMap.put("PrintCube", new PrintCommand("Cube"));
      eventMap.put("PrintStored", new PrintCommand("Stored"));
      eventMap.put("PrintOne", new PrintCommand("one"));
      eventMap.put("PrintTwo", new PrintCommand("two"));
      eventMap.put("PrintEnd", new PrintCommand("end"));
      eventMap.put("Reset Field Orientation", new InstantCommand(drivetrain::resetFieldOrientation));
    }

    autoPaths = new PPRobotPath[] {
      null,
      new PPRobotPath("New Path", drivetrain, false, eventMap),
      new PPRobotPath("3 game piece", drivetrain, false, eventMap),
      new PPRobotPath("Near Loading Zone 2 Game Piece + Balance", drivetrain, false, eventMap),
      new PPRobotPath("Near Loading Zone 3 Game Piece", drivetrain, false, eventMap),
      new PPRobotPath("TESTING", drivetrain, false, eventMap),
      new PPRobotPath("Basic", drivetrain, false, eventMap),
      new PPRobotPath("Basic 2", drivetrain, false, eventMap),
      new PPRobotPath("Mid Basic", drivetrain, false, eventMap),
      new PPRobotPath("Basic 3", drivetrain, false, eventMap)
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

    roller.setDefaultCommand(new HoldRoller(roller));

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

    new JoystickButton(driverController, Driver.rotateToFieldRelativeAngle0Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(0), drivetrain));
    new JoystickButton(driverController, Driver.rotateToFieldRelativeAngle90Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(-90), drivetrain));
    new JoystickButton(driverController, Driver.rotateToFieldRelativeAngle180Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(180), drivetrain));
    new JoystickButton(driverController, Driver.rotateToFieldRelativeAngle270Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));
  }

  private void configureButtonBindingsManipulator() {
    BooleanSupplier isCube = () -> new JoystickButton(manipulatorController, Manipulator.toggleCubeButton).getAsBoolean();
    BooleanSupplier isFront = () -> new JoystickButton(manipulatorController, Manipulator.toggleFrontButton).getAsBoolean();
    BooleanSupplier isIntake = () -> !isCube.getAsBoolean();
    // BooleanSupplier isFront = () -> false;
    //BooleanSupplier isStopped = () -> new JoystickButton(manipulatorController, Manipulator.stopRollerButton).getAsBoolean();

    new JoystickButton(manipulatorController, Manipulator.storePosButton).onTrue(new SetArmWristGoalPreset(GoalPos.STORED, isCube, isFront, arm));
    new JoystickButton(manipulatorController, Manipulator.lowPosButton).onTrue(new SetArmWristGoalPreset(GoalPos.LOW, isCube, isFront, arm));
    new JoystickButton(manipulatorController, Manipulator.midPosButton).onTrue(new SetArmWristGoalPreset(GoalPos.MID, isCube, isFront, arm));
    new JoystickButton(manipulatorController, Manipulator.highPosButton).onTrue(new SetArmWristGoalPreset(GoalPos.HIGH, isCube, isFront, arm));
    new POVButton(manipulatorController, Manipulator.shelfPickupPOV).onTrue(new SetArmWristGoalPreset(GoalPos.SHELF, isCube, isFront, arm));
    new POVButton(manipulatorController, Manipulator.intakeConePOV).onTrue(new SetArmWristGoalPreset(GoalPos.INTAKE, () -> false, isFront, arm).andThen(new RunRoller(roller, RollerMode.INTAKE_CONE, Constants.Roller.conePickupColor)));
    new POVButton(manipulatorController, Manipulator.substationPickupPOV).onTrue(new SetArmWristGoalPreset(GoalPos.SUBSTATION, isCube, isFront, arm).andThen(new RunRoller(roller, RollerMode.INTAKE_CONE, Constants.Roller.conePickupColor)));
    new POVButton(manipulatorController, Manipulator.intakeCubePOV).onTrue(new SetArmWristGoalPreset(GoalPos.INTAKE, () -> true, isFront, arm).andThen(new RunRoller(roller, RollerMode.INTAKE_CUBE, Constants.Roller.cubePickupColor)));
    new JoystickButton(manipulatorController, Manipulator.stopRollerButton).onTrue(new RunRoller(roller, RollerMode.STOP, Constants.Roller.defaultColor));
    // axisTrigger(manipulatorController, Manipulator.rollerIntakeConeButton)
    //   .onTrue(new RunRoller(roller, RollerMode.INTAKE_CONE, Constants.Roller.conePickupColor));
    axisTrigger(manipulatorController, Manipulator.rollerIntakeCubeButton)
      .onTrue(new ConditionalCommand(
        new RunRoller(roller, RollerMode.INTAKE_CUBE, Constants.Roller.cubePickupColor),
        new RunRoller(roller, RollerMode.OUTTAKE_CUBE, Constants.Roller.defaultColor),
        isIntake
      ));
    axisTrigger(manipulatorController, Manipulator.rollerIntakeConeButton)
      .onTrue(new ConditionalCommand(
        new RunRoller(roller, RollerMode.INTAKE_CONE, Constants.Roller.conePickupColor),
        new RunRoller(roller, RollerMode.OUTTAKE_CONE, Constants.Roller.defaultColor),
        isIntake
      ));

  }

  public Command stopDt() {
    return (new InstantCommand(drivetrain::stop).repeatedly()).until(drivetrain::isStopped);
  }

  public Command getAutonomousCommand() {
    // PPRobotPath autoPath = new PPRobotPath("New Path", drivetrain, false, new HashMap<>());
    Command[] autoPath = {
      new PPRobotPath("Mid Basic 3", drivetrain, false, eventMap).getPathCommand(true, true),
      new PPRobotPath("Mid Basic 4", drivetrain, false, eventMap).getPathCommand(false, true)
    };
    Command[] commands = {
      stopDt(),
      new WaitCommand(0)
    };
    SequentialCommandGroup autoCommand = new SequentialCommandGroup();
    for (int i = 0; i < autoPath.length; i++) {
      autoCommand.addCommands(autoPath[i]);
      autoCommand.addCommands(commands[i]);
    }

    // for(int i = 0; i < autoSelectors.length; i++) {
    //   if(!autoSelectors[i].get()) {
    //     System.out.println("Using Path: " + i);
    //     autoPath = autoPaths[i];
    //     break;
    //   }
    // }

    return autoPath == null ? new PrintCommand("No Autonomous Routine selected") : autoCommand;
    // return autoPath == null ? new PrintCommand("null :(") : autoPath.getPathCommand(true, true);
  }

  public void onEnable() {
    lime.getNTEntry("pipeline").setDouble(DriverStation.getAlliance() == Alliance.Red ? 1 : 0);
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
