package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class RotateToFieldRelativeAngle extends ProxyCommand {

    public RotateToFieldRelativeAngle(Rotation2d angle, Drivetrain drivetrain) {
        super(() ->
            new DriveToPoint(
                new Pose2d(drivetrain.getPose().getTranslation(), angle),
                drivetrain)
        );
        addRequirements(drivetrain);
    }

}
