// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import java.util.function.BooleanSupplier;

import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;
import org.carlmontrobotics.robotcode2023.subsystems.Roller;
import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.CUBE;
import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.CONE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToScoringPosition extends InstantCommand {
  /** Creates a new AlignToScoringPosition. */
  // TODO: Turning automatically to face goal. Currently it assumes that the robot is already facing or facing opposite the goal
  public AlignToScoringPosition(BooleanSupplier isCube, Roller roller, Drivetrain drivetrain) {
    super(() -> 
      {
        Translation2d goal;
        if (isCube.getAsBoolean())
          goal = drivetrain.getNearestGoal(CUBE, DriverStation.getAlliance());
        else
          goal = drivetrain.getNearestGoal(CONE, DriverStation.getAlliance());
        goal.plus(roller.correctPosition());
        CommandScheduler.getInstance().schedule(new DriveToPoint(() -> new Pose2d(goal, drivetrain.getPose().getRotation()), drivetrain));
      }
    );
  }
}
