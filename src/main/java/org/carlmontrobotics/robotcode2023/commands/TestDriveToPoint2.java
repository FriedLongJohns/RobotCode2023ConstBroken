// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestDriveToPoint2 extends SequentialCommandGroup {
  private static double testDistX = 0;
  private static double testDistY = 0;
  public TestDriveToPoint2(Drivetrain dt) {
    //using smartDashboard determined values, robot first drives X poseX determined meters, then drives Y poseY determined meters

    super(
      new InstantCommand(() -> {
          Pose2d position = dt.getPose();
          //goes DistY 0 degrees from original angle
          Translation2d xTrans = new Translation2d(testDistX, position.getRotation());
          Transform2d transformX = new Transform2d(xTrans, new Rotation2d(0));
          Pose2d poseX = position.plus(transformX);
          CommandScheduler.getInstance().schedule(new DriveToPoint(()->poseX, dt));
        }
      ),
      new InstantCommand(() -> {
          Pose2d position2 = dt.getPose();
          //goes DistY 90 degrees from original angle
          Translation2d yTrans = new Translation2d(testDistY, position2.getRotation());
          Transform2d transformY = new Transform2d(yTrans, new Rotation2d(Math.PI/2));
          Pose2d poseY = position2.plus(transformY);
          CommandScheduler.getInstance().schedule(new DriveToPoint(()->poseY, dt));
          // Use addRequirements() here to declare subsystem dependencies.
        }
      )
    );
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("testDistX", () -> testDistX, x -> testDistX = x);
      builder.addDoubleProperty("testDistY",   () -> testDistY, y -> testDistY = y);
  }
}
