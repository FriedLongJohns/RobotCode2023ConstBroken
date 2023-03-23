// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import java.util.function.BooleanSupplier;

import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;
import org.carlmontrobotics.robotcode2023.subsystems.Roller;
import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.CUBE;
import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.CONE;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToScoringPosition extends ConditionalCommand {
  /** Creates a new AlignToScoringPosition. */
  public AlignToScoringPosition(BooleanSupplier isCube, Roller roller, Drivetrain drivetrain) {
    super(
        new SequentialCommandGroup(
          new DriveToPoint(() -> drivetrain.getNearestGoal(CUBE, DriverStation.getAlliance()), drivetrain),
          new DriveToPoint(roller::correctPosition, drivetrain)
        ),
        new SequentialCommandGroup(
          new DriveToPoint(() -> drivetrain.getNearestGoal(CONE, DriverStation.getAlliance()), drivetrain),
          new DriveToPoint(roller::correctPosition, drivetrain)
        ),
        isCube
      );
  }
}
