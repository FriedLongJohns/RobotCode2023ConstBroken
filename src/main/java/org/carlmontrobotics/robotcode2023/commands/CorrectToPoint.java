// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.*;

import java.util.function.Supplier;

import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CorrectToPoint extends CommandBase {
  private Drivetrain dt;
  private PIDController pidX;
  private PIDController pidY;
  private PIDController pidTheta;

  /** Creates a new CorrectToPoint. */
  public CorrectToPoint(Supplier<Pose2d> setpoint, Drivetrain dt) {
    addRequirements(this.dt = dt);

    pidX     = new PIDController( dt.getPIDConstants()[0][0], dt.getPIDConstants()[0][1], dt.getPIDConstants()[0][2]);
    pidY     = new PIDController( dt.getPIDConstants()[1][0], dt.getPIDConstants()[1][1], dt.getPIDConstants()[1][2]);
    pidTheta = new PIDController( dt.getPIDConstants()[2][0], dt.getPIDConstants()[2][1], dt.getPIDConstants()[2][2]);

    pidX.setTolerance(positionTolerance[0], velocityTolerance[0]);
    pidY.setTolerance(positionTolerance[1], velocityTolerance[1]);
    pidTheta.setTolerance(positionTolerance[2], velocityTolerance[2]);
    pidTheta.enableContinuousInput(-180, 180);

    pidX.setSetpoint(setpoint.get().getX());
    pidY.setSetpoint(setpoint.get().getY());
    pidTheta.setSetpoint(setpoint.get().getRotation().getDegrees()); // The PIDController should correct for continuous input
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.drive(pidX.calculate(dt.getPose().getX()),
             pidY.calculate(dt.getPose().getY()),
             pidTheta.calculate(dt.getHeadingDeg()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidX.atSetpoint() && pidY.atSetpoint() && pidTheta.atSetpoint();
  }
}
