package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.*;

import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetArmWristPositionV2 extends SequentialCommandGroup {

    public SetArmWristPositionV2(double armPos, double wristPos, Arm arm) {
        super(
            // Move the arm to a point where we can safely fold the wrist
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> arm.setArmTarget(ARM_VERTICAL_POS + Math.copySign(MIN_WRIST_FOLD_POS, arm.getArmPos() - ARM_VERTICAL_POS), 0)),
                    new WaitUntilCommand(arm::armAtSetpoint)
                ),
                new InstantCommand(),
                () -> Math.abs(arm.getArmPos() - ARM_VERTICAL_POS) < MIN_WRIST_FOLD_POS
            ),

            // Fold the wrist
            new InstantCommand(() -> arm.setWristTarget(wristStowPos, 0)),
            new WaitUntilCommand(arm::wristAtSetpoint),

            // Move the arm to a point where we can safely unfold the wrist
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> arm.setArmTarget(ARM_VERTICAL_POS + Math.copySign(MIN_WRIST_FOLD_POS, arm.getArmPos() - ARM_VERTICAL_POS), 0)),
                    new WaitUntilCommand(arm::armAtSetpoint),

                    // Unfold the wrist
                    new InstantCommand(() -> arm.setWristTarget(wristPos, 0)),
                    new WaitUntilCommand(arm::wristAtSetpoint),

                    // Move the arm to the goal position
                    new InstantCommand(() -> arm.setArmTarget(armPos, 0)),
                    new WaitUntilCommand(arm::armAtSetpoint)
                ),
                new SequentialCommandGroup(
                    // If the goal position already satisfies this requirement move there
                    new InstantCommand(() -> arm.setArmTarget(armPos, 0)),
                    new WaitUntilCommand(arm::armAtSetpoint),

                    // Unfold the wrist
                    new InstantCommand(() -> arm.setWristTarget(wristPos, 0)),
                    new WaitUntilCommand(arm::wristAtSetpoint)
                ),
                () -> Math.abs(armPos - ARM_VERTICAL_POS) < MIN_WRIST_FOLD_POS
            )
        );
    }

}
