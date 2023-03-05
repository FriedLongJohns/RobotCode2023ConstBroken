// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.tolerance;

public class CorrectToPoint extends CommandBase {
  /** Creates a new CorrectToPoint. */
  private Drivetrain dt;
  private Pose2d setpoint;
  private PIDController pidX;
  private PIDController pidY;
  private PIDController pidTheta;
  public CorrectToPoint(Pose2d setpoint, Drivetrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt = dt);
    this.setpoint = setpoint;
    pidX     = new PIDController( dt.getPIDConstants()[0][0], dt.getPIDConstants()[0][1], dt.getPIDConstants()[0][2]);
    pidY     = new PIDController( dt.getPIDConstants()[1][0], dt.getPIDConstants()[1][1], dt.getPIDConstants()[1][2]);
    pidTheta = new PIDController( dt.getPIDConstants()[2][0], dt.getPIDConstants()[2][1], dt.getPIDConstants()[2][2]);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Target X", setpoint.getX());
    SmartDashboard.putNumber("Target Y", setpoint.getY());
    SmartDashboard.putNumber("Target Theta", setpoint.getRotation().getDegrees());
    dt.drive(pidX.calculate(dt.getPose().getX(), setpoint.getX()), 
             pidY.calculate(dt.getPose().getY(), setpoint.getY()),
             pidTheta.calculate(dt.getHeadingDeg(), setpoint.getRotation().getDegrees())); // TODO: make it turn -90 deg when told to turn 270 deg
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double diffX = Math.abs(dt.getPose().getX() - setpoint.getX());
    double diffY = Math.abs(dt.getPose().getY() - setpoint.getY());
    double diffTheta = Math.abs(dt.getHeadingDeg() - setpoint.getRotation().getDegrees());
    return diffX < tolerance[0] && diffY < tolerance[1] && diffTheta < tolerance[2] && dt.getSpeeds().vxMetersPerSecond < tolerance[0] && dt.getSpeeds().vyMetersPerSecond < tolerance[1] && Units.radiansToDegrees(dt.getSpeeds().omegaRadiansPerSecond) < tolerance[2];
  }
}
