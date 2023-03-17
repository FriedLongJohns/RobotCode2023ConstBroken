// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import java.util.function.DoubleSupplier;

import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmPeriodic extends CommandBase {
  /** Creates a new ArmPeriodic. */
  private Arm armSubsystem;
  private DoubleSupplier arm;
  private DoubleSupplier wrist;
  private double currentArm = 0, currentWrist = 0; // current velocities

  public ArmPeriodic(Arm armSubsystem, DoubleSupplier arm, DoubleSupplier wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.armSubsystem = armSubsystem);
    this.arm = arm;
    this.wrist = wrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] speeds = getRequestedSpeeds();
    // if driven by joysticks, run only feedforward
    if (speeds[ARM] == 0) 
      armSubsystem.driveArm(0, 0, true);
    else {
      armSubsystem.driveArm(speeds[ARM], 0, false);
      armSubsystem.setArmTarget(armSubsystem.getArmPos());
    }
    if (speeds[WRIST] == 0)
      armSubsystem.driveWrist(0, 0, true);
    else {
      armSubsystem.driveWrist(speeds[WRIST], 0, false);
      armSubsystem.setWristTarget(armSubsystem.getWristPos());
    }
  }

  // Copy and pasted from drivetrain, handles input from joysticks
  public double[] getRequestedSpeeds() {
    double rawArm, rawWrist, deltaT;
    deltaT = .05;
    // Sets all values less than or equal to a very small value (determined by the
    // idle joystick state) to zero.
    // Used to make sure that the robot does not try to change its angle unless it
    // is moving,
    if (Math.abs(arm.getAsDouble()) <= Constants.OI.JOY_THRESH)
      rawArm = 0.0;
    else
      rawArm = MAX_FF_VEL[ARM] * arm.getAsDouble();
    if (Math.abs(wrist.getAsDouble()) <= Constants.OI.JOY_THRESH)
      rawWrist = 0.0;
    else
      rawWrist = MAX_FF_VEL[WRIST] * wrist.getAsDouble();

    double targetAccelerationArm = (rawArm - currentArm) / deltaT;
    double targetAccelerationWrist = (rawWrist - currentWrist) / deltaT;
    if (targetAccelerationArm >= MAX_FF_ACCEL[ARM]) {
      targetAccelerationArm *= MAX_FF_ACCEL[ARM] / targetAccelerationArm;
    }
    if (targetAccelerationWrist >= MAX_FF_ACCEL[WRIST]) {
      targetAccelerationWrist *= MAX_FF_ACCEL[WRIST] / targetAccelerationWrist;
    }
    currentArm += targetAccelerationArm * deltaT;
    currentWrist += targetAccelerationWrist * deltaT;
    if (Math.abs(currentArm) <= Constants.OI.JOY_THRESH)
      currentArm = 0;
    if (Math.abs(currentWrist) <= Constants.OI.JOY_THRESH)
      currentWrist = 0;
    return new double[] { currentArm, currentWrist };
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
