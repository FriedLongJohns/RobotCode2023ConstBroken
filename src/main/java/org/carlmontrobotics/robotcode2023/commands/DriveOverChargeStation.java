// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveOverChargeStation extends CommandBase {

  private final Drivetrain drivetrain;
  private boolean fwd, fieldOriented;
  private boolean goingUp, goingDown, overChargeStation;
  private int upSign;

  public DriveOverChargeStation(Drivetrain drivetrain) {
    addRequirements(this.drivetrain = drivetrain);
  }

  @Override
  public void initialize() {
    fwd = Math.abs(getRoll()) > Math.abs(getPitch());
    fieldOriented = drivetrain.getFieldOriented();
    drivetrain.setFieldOriented(false);
  }

  @Override
  public void execute() {
    double roll = getRoll();
    double pitch = getPitch();
    double angle = fwd ? roll : pitch;
    if (!goingUp) {
      goingUp = Math.abs(angle) > 8;
      upSign = angle >= 0 ? 1 : -1;
    }
    if (goingUp && !goingDown && upSign != 0) {

      goingDown = (upSign > 0 ? angle < 0 : angle > 0) && Math.abs(angle) > 8;
    }
    double forward = fwd ? drivetrain.getMaxSpeedMps() : 0;
    double strafe = fwd ? 0 : drivetrain.getMaxSpeedMps();
    drivetrain.drive(forward, strafe, 0);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    drivetrain.setFieldOriented(fieldOriented);
  }

  @Override
  public boolean isFinished() {
    double roll = -getRoll();
    double pitch = getPitch();
    double angle = fwd ? roll : pitch;
    return goingDown && Math.abs(angle) > 8;
  }

  private double getPitch() {
    return drivetrain.getPitch() - drivetrain.initPitch;
  }

  private double getRoll() {
    return drivetrain.getRoll() - drivetrain.initRoll;
  }
}
