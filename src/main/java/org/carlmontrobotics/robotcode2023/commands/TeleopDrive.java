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
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {

  private final Drivetrain drivetrain;
  private DoubleSupplier fwd;
  private DoubleSupplier str;
  private DoubleSupplier rcw;
  private BooleanSupplier slow;
  double currentForward = 0;
  double currentStrafe = 0;
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

  public double[] getRequestedSpeeds() {double rawForward, rawStrafe, rotateClockwise, deltaT;
    deltaT = .05;
    // Sets all values less than or equal to a very small value (determined by the idle joystick state) to zero.
    // Used to make sure that the robot does not try to change its angle unless it is moving,
    if (Math.abs(fwd.getAsDouble()) <= Constants.OI.JOY_THRESH) rawForward = 0.0;
    else rawForward = maxForward * fwd.getAsDouble();
    if (Math.abs(str.getAsDouble()) <= Constants.OI.JOY_THRESH) rawStrafe = 0.0;
    else rawStrafe = maxStrafe * str.getAsDouble();

    rotateClockwise = rcw.getAsDouble();
    if (Math.abs(rotateClockwise) <= Constants.OI.JOY_THRESH) rotateClockwise = 0.0;
    else rotateClockwise = maxRCW * rotateClockwise;
    double targetAccelerationX = (rawForward - currentForward)/deltaT;
    double targetAccelerationY = (rawStrafe - currentStrafe)/deltaT;
    double accelerationMagnitude = Math.hypot(targetAccelerationX, targetAccelerationY);
    if (accelerationMagnitude >= autoMaxAccelMps2) {
      targetAccelerationX *= autoMaxAccelMps2/accelerationMagnitude;
      targetAccelerationY *= autoMaxAccelMps2/accelerationMagnitude;
    }
    currentForward += targetAccelerationX*deltaT;
    currentStrafe += targetAccelerationY*deltaT;
    if (Math.abs(currentForward) <= Constants.OI.JOY_THRESH)
      currentForward = 0;
    if (Math.abs(currentStrafe) <= Constants.OI.JOY_THRESH)
      currentStrafe = 0;
    double driveMultiplier = slow.getAsBoolean() ? kSlowDriveSpeed : 1;
    double rotationMultiplier = slow.getAsBoolean() ? kSlowDriveRotation : 0.55;
    return new double[] {currentForward * driveMultiplier, currentStrafe * driveMultiplier, -rotateClockwise * rotationMultiplier};
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
