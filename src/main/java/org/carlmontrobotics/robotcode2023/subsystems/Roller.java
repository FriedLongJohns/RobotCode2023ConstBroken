// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.subsystems;

import java.awt.Color;
import java.util.Arrays;

import org.carlmontrobotics.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import static org.carlmontrobotics.robotcode2023.Constants.Roller.*;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Roller extends SubsystemBase {

    private final CANSparkMax motor = MotorControllerFactory.createSparkMax(rollerPort, MotorConfig.NEO_550);
    private TimeOfFlight distSensor = new TimeOfFlight(10);
    private final AddressableLED led = new AddressableLED(ledPort);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledLength);
    private boolean hadGamePiece = false;
    private Drivetrain dt;

    private Command resetColorCommand = new SequentialCommandGroup(
            new WaitCommand(ledDefaultColorRestoreTime),
            new InstantCommand(() -> setLedColor(defaultColor))) {
        public boolean runsWhenDisabled() {
            return true;
        };
    };

    public Roller(Drivetrain dt) {
        led.setLength(ledBuffer.getLength());
        setLedColor(defaultColor);
        // SmartDashboard.putData(this);
        this.dt = dt;
        led.start();
    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("Has Game Piece", hasGamePiece());
        SmartDashboard.putNumber("Roller Game Piece Distance", getGamePieceDistanceIn());

        // LED Update
        {
            boolean hasGamePiece = hasGamePiece();
            if (hasGamePiece && !hadGamePiece) {
                setLedColor(pickupSuccessColor);
                resetColorCommand.schedule();
            }
            hadGamePiece = hasGamePiece;
        }
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void setLedColor(Color color) {
        for (int i = 0; i < ledBuffer.getLength(); i++)
            ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        led.setData(ledBuffer);
    }

    public boolean hasGamePiece() {
        // return false;
        return getGamePieceDistanceIn() < gamePieceDetectDistanceIn;
    }

    public double getGamePieceDistanceIn() {
        return Units.metersToInches((distSensor.getRange() - distSensorDepthMM /*
                                                                                * The sensor measures from the back of
                                                                                * the sensor
                                                                                */) / 1000 /* Convert mm to m */);
    }

    // This assumes robot's middle is exactly in line with the poles where you place
    // the cones
    // outtake side is the same side as the limelight
    public Pose2d correctPosition() {
        Pose2d initialPose = dt.getPose();
        double distanceToMove = 0;
        double dist = getGamePieceDistanceIn();
        Translation2d translation = new Translation2d(0, 0);
        if (dist < LEFT_LIMIT) {
            // need to shift robot right
            distanceToMove = ROLLER_WIDTH / 2 - dist;
            translation = new Translation2d(distanceToMove,
                    initialPose.getRotation().plus(new Rotation2d(Math.PI / 2)));

        } else if (dist > RIGHT_LIMIT) {
            // need to shift robot left
            distanceToMove = dist - ROLLER_WIDTH / 2;
            translation = new Translation2d(distanceToMove,
                    initialPose.getRotation().plus(new Rotation2d(-Math.PI / 2)));
        }
        Transform2d transform = new Transform2d(translation, new Rotation2d(0));
        return initialPose.plus(transform);
    }

    public Pose2d getNearestGoal(boolean mode)
    {
        Pose2d goal = dt.getPose();
        if (mode) {
            goal = goal.nearest(Arrays.asList(cubeScoringPos));
        } else {
            goal = goal.nearest(Arrays.asList(coneScoringPos));
        }
        return goal;
    }

    public void putRollerConstsOnSmartDashboard() {
        SmartDashboard.putNumber("Intake Cone Speed", RollerMode.INTAKE_CONE.speed);
        SmartDashboard.putNumber("Outtake Cone Speed", RollerMode.OUTTAKE_CONE.speed);
        SmartDashboard.putNumber("Intake Cube Speed", RollerMode.INTAKE_CUBE.speed);
        SmartDashboard.putNumber("Outtake Cube Speed", RollerMode.OUTTAKE_CUBE.speed);
        SmartDashboard.putNumber("Intake Cone Time", RollerMode.INTAKE_CONE.time);
        SmartDashboard.putNumber("Outtake Cone Time", RollerMode.OUTTAKE_CONE.time);
        SmartDashboard.putNumber("Intake Cube Time", RollerMode.INTAKE_CUBE.time);
        SmartDashboard.putNumber("Outtake Cube Time", RollerMode.OUTTAKE_CUBE.time);
    }

    public void getRollerConstsOnSmartDashboard() {
        RollerMode.INTAKE_CONE.speed = SmartDashboard.getNumber("Intake Cone Speed", RollerMode.INTAKE_CONE.speed);
        RollerMode.OUTTAKE_CONE.speed = SmartDashboard.getNumber("Outtake Cone Speed", RollerMode.OUTTAKE_CONE.speed);
        RollerMode.INTAKE_CUBE.speed = SmartDashboard.getNumber("Intake Cube Speed", RollerMode.INTAKE_CUBE.speed);
        RollerMode.OUTTAKE_CUBE.speed = SmartDashboard.getNumber("Outtake Cube Speed", RollerMode.OUTTAKE_CUBE.speed);
        RollerMode.INTAKE_CONE.time = SmartDashboard.getNumber("Intake Cone Time", RollerMode.INTAKE_CONE.time);
        RollerMode.OUTTAKE_CONE.time = SmartDashboard.getNumber("Outtake Cone Time", RollerMode.OUTTAKE_CONE.time);
        RollerMode.INTAKE_CUBE.time = SmartDashboard.getNumber("Intake Cube Time", RollerMode.INTAKE_CUBE.time);
        RollerMode.OUTTAKE_CUBE.time = SmartDashboard.getNumber("Outtake Cube Time", RollerMode.OUTTAKE_CUBE.time);
    }
}
