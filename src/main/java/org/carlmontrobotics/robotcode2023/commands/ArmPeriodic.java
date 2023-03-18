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
    double[] speeds = getRequestedSpeeds();

    double goalArmRad = armSubsystem.getArmPos() + speeds[ARM] * deltaT;
    double goalWristRad = armSubsystem.getWristPos() + speeds[WRIST] * deltaT;

    TrapezoidProfile.Constraints armConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL[ARM], MAX_FF_ACCEL[ARM]);
    var armProfile = new TrapezoidProfile(armConstraints,
        new TrapezoidProfile.State(goalArmRad, speeds[ARM]),
        new TrapezoidProfile.State(currArmRad, armSubsystem.getArmVel()));

    TrapezoidProfile.Constraints wristConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL[WRIST], MAX_FF_ACCEL[WRIST]);
    var wristProfile = new TrapezoidProfile(wristConstraints,
        new TrapezoidProfile.State(goalWristRad, speeds[WRIST]),
        new TrapezoidProfile.State(currWristRad, armSubsystem.getWristVel()));

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    TrapezoidProfile.State armSetpoint = armProfile.calculate(deltaT);
    TrapezoidProfile.State wristSetpoint = wristProfile.calculate(deltaT);
    armSetpoint.position = armSubsystem.getArmClampedGoal(armSetpoint.position);
    wristSetpoint.position = armSubsystem.getWristClampedGoal(wristSetpoint.position);
    armSubsystem.setArmTarget(armSetpoint.position, armSetpoint.velocity);
    armSubsystem.setWristTarget(wristSetpoint.position, wristSetpoint.velocity);

    lastTime = currTime;
  }

  // Copy and pasted from drivetrain, handles input from joysticks
  public double[] getRequestedSpeeds() {
    double rawArmVel, rawWristVel;
    // Sets all values less than or equal to a very small value (determined by the
    // idle joystick state) to zero.
    if (Math.abs(arm.getAsDouble()) <= Constants.OI.JOY_THRESH)
      rawArmVel = 0.0;
    else
      rawArmVel = MAX_FF_VEL[ARM] * arm.getAsDouble();

    if (Math.abs(wrist.getAsDouble()) <= Constants.OI.JOY_THRESH)
      rawWristVel = 0.0;
    else
      rawWristVel = MAX_FF_VEL[WRIST] * wrist.getAsDouble();
    
    return new double[] {rawArmVel, rawWristVel};
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
