package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.wristStowPos;

import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetArmWristPosition extends SequentialCommandGroup {

    public SetArmWristPosition(double armPos, double wristPos, Arm arm) {
        // TODO: Alex simplify this command
        super(
            new InstantCommand(() -> arm.setWristTarget(wristStowPos)),
            new WaitUntilCommand(arm::wristAtSetpoint),
            new InstantCommand(() -> arm.setArmTarget(armPos)),
            new WaitUntilCommand(arm::armAtSetpoint),
            new InstantCommand(() -> arm.setWristTarget(wristPos)),
            new WaitUntilCommand(arm::wristAtSetpoint)
        );
        addRequirements(arm);
    }

}
