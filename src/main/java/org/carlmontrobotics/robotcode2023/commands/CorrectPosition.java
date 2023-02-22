// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.subsystems.DistanceSensor;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CorrectPosition extends CommandBase {
  /** Creates a new CorrectPosition. */

  // see DistanceSensor subsystem
  private int errorType;
  private DistanceSensor sensor;
  private Drivetrain dt;
  private Limelight lime;

  
  public CorrectPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.sensor = new DistanceSensor());
    addRequirements(this.dt = new Drivetrain(lime));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // THIS MIGHT NOT BELONG IN EXECUTE(). Please tell me if this is the case :)

    //could you swtich-case but whatever
    double dist = sensor.getDistance();
    errorType = sensor.getErrorType();
    Pose2d initialPose = dt.getPose();
    Pose2d finalPose;
    double distanceToMove;
    if (errorType == 1) {
      distanceToMove = (Constants.Roller.acceptableLeftLimit) - dist + 2; // added 2 to make sure that the new position is fine
      finalPose = new Pose2d(initialPose.getX() - Units.inchesToMeters(distanceToMove), initialPose.getY(), initialPose.getRotation());
      CommandScheduler.getInstance().schedule(new DriveToPoint(finalPose, dt));
    }
    else if (errorType == 2) {
      distanceToMove = dist - (Constants.Roller.acceptableRightLimit) + 2; // added 2 to make sure that the new position is fine
      finalPose = new Pose2d(initialPose.getX() + Units.inchesToMeters(distanceToMove), initialPose.getY(), initialPose.getRotation());
      CommandScheduler.getInstance().schedule(new DriveToPoint(finalPose, dt));
    }
    else if (errorType == 0) {
      // do nothing
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
