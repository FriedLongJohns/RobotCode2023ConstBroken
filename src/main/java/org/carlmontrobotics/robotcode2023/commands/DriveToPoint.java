package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.autoMaxAccelMps2;
import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.maxSpeed;

import java.util.HashMap;
import java.util.function.Supplier;

import org.carlmontrobotics.lib199.path.PPRobotPath;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveToPoint extends SequentialCommandGroup {

    public DriveToPoint(Supplier<Pose2d> targetPose, Drivetrain drivetrain) {
        super(
            // Use ProxyCommand so path is regenerated at runtime
            new ProxyCommand(() ->
                new PPRobotPath(
                    PathPlanner.generatePath(
                        new PathConstraints(maxSpeed, autoMaxAccelMps2),
                        PathPoint.fromCurrentHolonomicState(
                            drivetrain.getPose(),
                            drivetrain.getSpeeds()
                        ),
                        PathPoint.fromCurrentHolonomicState(
                            targetPose.get(),
                            new ChassisSpeeds()
                        )
                    ),
                    drivetrain,
                    new HashMap<>()
                ).getPathCommand(true, true)
            ),
            new CorrectToPoint(targetPose.get(), drivetrain)
        );
        addRequirements(drivetrain);
    }

}
