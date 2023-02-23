// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.subsystems;

import com.playingwithfusion.TimeOfFlight;

import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.commands.DriveToPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DistanceSensor extends SubsystemBase {

  /* constructor parameter: sensorID
  Methods:
  getRange() (gets distance)
  setRangingMode(rangingMode, sampleTime)
  getAmbientLightLevel()
  getStatus()




  */
  private Limelight lime;
  private Drivetrain dt = new Drivetrain(lime);
  private TimeOfFlight distSensor = new TimeOfFlight(10);
  double dist;
  /** Creates a new DistanceSensor. */
  public DistanceSensor() {
    SmartDashboard.putNumber("distance", distSensor.getRange());
    SmartDashboard.putNumber("light", distSensor.getAmbientLightLevel());
    SmartDashboard.putString("Status", distSensor.getStatus().toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double dist = Units.metersToInches(distSensor.getRange()/1000);
    SmartDashboard.putNumber("distance", dist);
    SmartDashboard.putNumber("light", distSensor.getAmbientLightLevel());
    SmartDashboard.putString("Status", distSensor.getStatus().toString());
  }

  public Constants.Roller.ErrorType getErrorType() {
    // We need the type of offset, if any:
    // MIDDLE means no adjustment, LEFT means we need to move the robot left and
    // RIGHT means we need to move the robot right. 

    if (dist < Constants.Roller.acceptableLeftLimit) return Constants.Roller.ErrorType.LEFT;
    else if (dist > Constants.Roller.acceptableRightLimit) return Constants.Roller.ErrorType.RIGHT;
    else return Constants.Roller.ErrorType.MIDDLE;
  }

  public double getDistance() {
    return dist;
  }

  public void correctPosition() {
    Constants.Roller.ErrorType errorType = getErrorType();
    Pose2d initialPose = dt.getPose();
    Pose2d finalPose;
    double distanceToMove;

    /* if (errorType == LEFT) {
      distanceToMove = (Constants.Roller.acceptableLeftLimit) - dist + 2; // added 2 to make sure that the new position is fine
      finalPose = new Pose2d(initialPose.getX() - Units.inchesToMeters(distanceToMove), initialPose.getY(), initialPose.getRotation());
      CommandScheduler.getInstance().schedule(new DriveToPoint(finalPose, dt));
    }
    else if (errorType == RIGHT) {
      distanceToMove = dist - (Constants.Roller.acceptableRightLimit) + 2; // added 2 to make sure that the new position is fine
      finalPose = new Pose2d(initialPose.getX() + Units.inchesToMeters(distanceToMove), initialPose.getY(), initialPose.getRotation());
      CommandScheduler.getInstance().schedule(new DriveToPoint(finalPose, dt));
    }
    else if (errorType == MIDDLE) {
      // do nothing
    } */

    switch (errorType) {
      case LEFT:
        distanceToMove = (Constants.Roller.acceptableLeftLimit) - dist + 2; // added 2 to make sure that the new position is fine
        /* finalPose = new Pose2d(initialPose.getX() + Units.inchesToMeters(distanceToMove), initialPose.getY(), initialPose.getRotation());
        CommandScheduler.getInstance().schedule(new DriveToPoint(finalPose, dt));
        */

        Translation2d translation = new Translation2d(distanceToMove, initialPose.getRotation().plus(new Rotation2d(Math.PI/2)));
        Transform2d transform = new Transform2d(translation, new Rotation2d(0));
        finalPose = initialPose.plus(transform);
        CommandScheduler.getInstance().schedule(new DriveToPoint(finalPose, dt));
      case RIGHT:
        distanceToMove = dist - (Constants.Roller.acceptableRightLimit) + 2; // added 2 to make sure that the new position is fine
        /*finalPose = new Pose2d(initialPose.getX() - Units.inchesToMeters(distanceToMove), initialPose.getY(), initialPose.getRotation());
        CommandScheduler.getInstance().schedule(new DriveToPoint(finalPose, dt));*/

        Translation2d translation2 = new Translation2d(distanceToMove, initialPose.getRotation().plus(new Rotation2d(-Math.PI/2)));
        Transform2d transform2 = new Transform2d(translation2, new Rotation2d(0));
        finalPose = initialPose.plus(transform2);
        CommandScheduler.getInstance().schedule(new DriveToPoint(finalPose, dt));
      case MIDDLE:
        // do nothing

    }

    // these are only temporary because the movements should be relavitve to the robot's orientation, not any particular axis.
  }
}