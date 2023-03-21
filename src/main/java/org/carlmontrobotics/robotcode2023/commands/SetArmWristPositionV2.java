package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.*;

import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetArmWristPositionV2 extends SequentialCommandGroup {

    public SetArmWristPositionV2(double armPos, double wristPos, Arm arm) {
        super(
            new ConditionalCommand(
                new SequentialCommandGroup(//Is wrist startpoint inside of the vertical drivetrain bounds? If yes, move arm to stow pos
                    new InstantCommand(() -> arm.setArmTarget(ARM_VERTICAL_POS + Math.copySign(MIN_WRIST_FOLD_POS, arm.getArmPos() - ARM_VERTICAL_POS), 0)),
                    new WaitUntilCommand(arm::armAtSetpoint)
                ),
                new InstantCommand(),//when false
                () -> isWristOutsideRobot(arm.getArmPos(), arm.getWristPos())
            ),

            new ConditionalCommand(//move wrist to closest stow position
                new InstantCommand(() -> arm.setWristTarget(WRIST_STOW_POS, 0)),//normal stow pos is closer
                new InstantCommand(() -> arm.setWristTarget(WRIST_NEG_STOW_POS, 0)),//neg stow pos is closer
                () -> arm.getWristPos() + ROLLER_COM_CORRECTION >= 0
            ),
            new WaitUntilCommand(arm::wristAtSetpoint),

            new InstantCommand(() -> arm.setArmTarget(ARM_VERTICAL_POS + Math.copySign(MIN_WRIST_FOLD_POS, armPos - ARM_VERTICAL_POS), 0)),
            new WaitUntilCommand(arm::armAtSetpoint),

            new ConditionalCommand(//move wrist to closest stow position
                new InstantCommand(() -> arm.setWristTarget(WRIST_STOW_POS, 0)),//normal stow pos is closer
                new InstantCommand(() -> arm.setWristTarget(WRIST_NEG_STOW_POS, 0)),//neg stow pos is closer
                () -> wristPos + ROLLER_COM_CORRECTION >= 0
            ),
            new WaitUntilCommand(arm::wristAtSetpoint),

            // Move the arm to a point where we can safely unfold the wrist to not collide with robot
            new ConditionalCommand(
                new SequentialCommandGroup(
                    // Unfold the wrist
                    new InstantCommand(() -> arm.setWristTarget(wristPos, 0)),
                    new WaitUntilCommand(arm::wristAtSetpoint),

                    new InstantCommand(() -> arm.setArmTarget(armPos, 0)),
                    new WaitUntilCommand(arm::armAtSetpoint)

                ),
                new SequentialCommandGroup(//Is wrist endpoint inside of the vertical drivetrain bounds? If yes, move arm to stow pos
                    // Move the arm to the goal position
                    new InstantCommand(() -> arm.setArmTarget(armPos, 0)),
                    new WaitUntilCommand(arm::armAtSetpoint),

                    // Unfold the wrist
                    new InstantCommand(() -> arm.setWristTarget(wristPos, 0)),
                    new WaitUntilCommand(arm::wristAtSetpoint)
                ),
                () -> isWristOutsideRobot(armPos, wristPos)
            )
        );
    }

    public static boolean isWristOutsideRobot(double armPos, double wristPos) {
        Translation2d wristTip = Arm.getWristTipPosition(armPos, wristPos);
        double driveTrainHalfLen = Units.inchesToMeters(31)/2;

        return Math.abs(wristTip.getX())>driveTrainHalfLen;
        //returns true if wristTip is outside of robot vertical bounds.
}

}
