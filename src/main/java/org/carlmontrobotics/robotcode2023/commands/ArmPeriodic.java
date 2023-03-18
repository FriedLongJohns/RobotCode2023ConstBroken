// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import java.util.function.DoubleSupplier;

import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmPeriodic extends CommandBase {
  /** Creates a new ArmPeriodic. */
  private Arm armSubsystem;
  private DoubleSupplier arm;
  private DoubleSupplier wrist;
  private double currArmRad = 0, currWristRad = 0;
  private double lastTime = 0;
  private final double EPSILON = 0.0001;

  public ArmPeriodic(Arm armSubsystem, DoubleSupplier arm, DoubleSupplier wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.armSubsystem = armSubsystem);
    this.arm = arm;
    this.wrist = wrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currArmRad = armSubsystem.getArmPos();
    currWristRad = armSubsystem.getWristPos();
    double currTime = Timer.getFPGATimestamp();
    double deltaT = currTime - lastTime;
    double[] goals = getRequestedSpeeds(deltaT);

    TrapezoidProfile.Constraints armConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL[ARM], MAX_FF_ACCEL[ARM]);
    var armProfile = new TrapezoidProfile(armConstraints,
        new TrapezoidProfile.State(goals[0], goals[1]),
        new TrapezoidProfile.State(currArmRad, armSubsystem.getArmPos()));

    TrapezoidProfile.Constraints wristConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL[WRIST], MAX_FF_ACCEL[WRIST]);
    var wristProfile = new TrapezoidProfile(wristConstraints,
        new TrapezoidProfile.State(goals[2], goals[3]),
        new TrapezoidProfile.State(currWristRad, armSubsystem.getWristPos()));

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    TrapezoidProfile.State armSetpoint = armProfile.calculate(deltaT);
    TrapezoidProfile.State wristSetpoint = wristProfile.calculate(deltaT);
    armSetpoint.position = armSubsystem.getArmClampedGoal(armSetpoint.position);
    wristSetpoint.position = armSubsystem.getWristClampedGoal(wristSetpoint.position);
    armSubsystem.driveArm(armSetpoint);
    armSubsystem.driveWrist(wristSetpoint);

    if (Math.abs(goals[1]) < EPSILON)
      armSubsystem.setArmTarget(armSubsystem.getArmPos());

    if (Math.abs(goals[3]) < EPSILON) {
      armSubsystem.setWristTarget(armSubsystem.getWristPos());
    }
    lastTime = currTime;
  }

  // Copy and pasted from drivetrain, handles input from joysticks
  public double[] getRequestedSpeeds(double deltaT) {
    double rawArmVel, rawWristVel;
    // Sets all values less than or equal to a very small value (determined by the
    // idle joystick state) to zero.
    // Used to make sure that the robot does not try to change its angle unless it
    // is moving,
    if (Math.abs(arm.getAsDouble()) <= Constants.OI.JOY_THRESH)
      rawArmVel = 0.0;
    else
      rawArmVel = MAX_FF_VEL[ARM] * arm.getAsDouble();

    if (Math.abs(wrist.getAsDouble()) <= Constants.OI.JOY_THRESH)
      rawWristVel = 0.0;
    else
      rawWristVel = MAX_FF_VEL[WRIST] * wrist.getAsDouble();

    double goalArmRad = armSubsystem.getArmPos() + rawArmVel * deltaT;
    double goalWristRad = armSubsystem.getWristPos() + rawWristVel * deltaT;

    return new double[] { goalArmRad, rawArmVel, goalWristRad, rawWristVel };
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
