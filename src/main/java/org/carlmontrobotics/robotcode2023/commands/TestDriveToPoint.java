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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestDriveToPoint extends InstantCommand {
  private static double testDistX = 0;
  private static double testDistY = 0;
  private static double testRadTheta = 0;
  public TestDriveToPoint(Drivetrain dt) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    // using smartDashboard, robot drive directly to (x, y)
    Pose2d position = dt.getPose();
    Translation2d xTrans = new Translation2d(testDistX, position.getRotation());
    Transform2d transformX = new Transform2d(xTrans, new Rotation2d(0));
    Pose2d pose = position.plus(transformX);
    Translation2d yTrans = new Translation2d(testDistY, position.getRotation());
    Transform2d transformY = new Transform2d(yTrans, new Rotation2d(Math.PI/2));
    pose = position.plus(transformY);
    final Pose2d goal = pose;
    CommandScheduler.getInstance().schedule(new DriveToPoint(() -> goal, dt));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("testDistX", () -> testDistX, x -> testDistX = x);
      builder.addDoubleProperty("testDistY",   () -> testDistY, y -> testDistY = y);
      builder.addDoubleProperty("testRadTheta", () -> testRadTheta, theta -> testRadTheta = theta);
  }
}
