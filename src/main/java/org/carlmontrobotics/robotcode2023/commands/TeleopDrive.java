/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.Robot;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {

  private static final double robotPeriod = Robot.robot.getPeriod();
  private final Drivetrain drivetrain;
  private DoubleSupplier fwd;
  private DoubleSupplier str;
  private DoubleSupplier rcw;
  private BooleanSupplier slow;
  private double currentForwardVel = 0;
  private double currentStrafeVel = 0;

  /**
   * Creates a new TeleopDrive.
   */
  public TeleopDrive(Drivetrain drivetrain, DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rcw, BooleanSupplier slow) {
    addRequirements(this.drivetrain = drivetrain);
    this.fwd = fwd;
    this.str = str;
    this.rcw = rcw;
    this.slow = slow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] speeds = getRequestedSpeeds();
    drivetrain.drive(speeds[0], speeds[1], speeds[2]);
  }

  public double[] getRequestedSpeeds() {
    // Sets all values less than or equal to a very small value (determined by the idle joystick state) to zero.
    // Used to make sure that the robot does not try to change its angle unless it is moving,
    double forward = fwd.getAsDouble();
    double strafe = str.getAsDouble();
    double rotateClockwise = rcw.getAsDouble();
    if (Math.abs(forward) <= Constants.OI.JOY_THRESH) forward = 0.0;
    else forward *= maxForward;
    if (Math.abs(strafe) <= Constants.OI.JOY_THRESH) strafe = 0.0;
    else strafe *= maxStrafe;
    if (Math.abs(rotateClockwise) <= Constants.OI.JOY_THRESH) rotateClockwise = 0.0;
    else rotateClockwise *= maxRCW;

    // Limit acceleration of the robot
    double accelerationX = (forward - currentForwardVel) / robotPeriod;
    double accelerationY = (strafe - currentStrafeVel) / robotPeriod;
    double translationalAcceleration = Math.hypot(accelerationX, accelerationY);
    if(translationalAcceleration > autoMaxAccelMps2) {
      Translation2d limitedAccelerationVector = new Translation2d(autoMaxAccelMps2, Rotation2d.fromRadians(Math.atan2(accelerationY, accelerationX)));
      Translation2d limitedVelocityVector = limitedAccelerationVector.times(robotPeriod);
      currentForwardVel += limitedVelocityVector.getX();
      currentStrafeVel += limitedVelocityVector.getY();
    } else {
      currentForwardVel = forward;
      currentStrafeVel = strafe;
    }

    // ATM, there is no rotational acceleration limit

    // If the above math works, no velocity should be greater than the max velocity, so we don't need to limit it.

    // Quick-Stop
    if (Math.abs(forward) <= minVelocityMps) currentForwardVel = 0;
    if (Math.abs(strafe) <= minVelocityMps) currentStrafeVel = 0;
    if (Math.abs(rotateClockwise) <= minRotationRadPSec) rotateClockwise = 0;

    double driveMultiplier = slow.getAsBoolean() ? kSlowDriveSpeed : kNormalDriveSpeed;
    double rotationMultiplier = slow.getAsBoolean() ? kSlowDriveRotation : kNormalDriveRotation;

    return new double[] {currentForwardVel * driveMultiplier, currentStrafeVel * driveMultiplier, -rotateClockwise * rotationMultiplier};
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
