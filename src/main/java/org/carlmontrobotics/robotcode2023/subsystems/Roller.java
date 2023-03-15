// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.subsystems;

import java.awt.Color;

import org.carlmontrobotics.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import static org.carlmontrobotics.robotcode2023.Constants.Roller.*;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;

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
    public double time = 1.5;

    private boolean hadGamePiece = false;

    private Command resetColorCommand = new SequentialCommandGroup(
        new WaitCommand(ledDefaultColorRestoreTime),
        new InstantCommand(() -> setLedColor(defaultColor)))
    {
        public boolean runsWhenDisabled() {
            return true;
        };
    };

    public Roller() {
        led.setLength(ledBuffer.getLength());
        setLedColor(defaultColor);
        SmartDashboard.putData(this);

        // TODO: Get proper values for the speeds and timings. For future pull requests, do not merge if this is not deleted
        SmartDashboard.putNumber("Intake Cone Speed", RollerMode.INTAKE_CONE.speed);
        SmartDashboard.putNumber("Outtake Cone Speed", RollerMode.OUTTAKE_CONE.speed);
        SmartDashboard.putNumber("Intake Cube Speed", RollerMode.INTAKE_CUBE.speed);
        SmartDashboard.putNumber("Outtake Cube Speed", RollerMode.OUTTAKE_CUBE.speed);
        SmartDashboard.putNumber("Intake Cone Time", RollerMode.INTAKE_CONE.time);
        SmartDashboard.putNumber("Outtake Cone Time", RollerMode.OUTTAKE_CONE.time);
        SmartDashboard.putNumber("Intake Cube Time", RollerMode.INTAKE_CUBE.time);
        SmartDashboard.putNumber("Outtake Cube Time", RollerMode.OUTTAKE_CUBE.time);
        led.start();
    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("Has Game Piece", hasGamePiece());
        RollerMode.INTAKE_CONE.speed = SmartDashboard.getNumber("Intake Cone Speed", RollerMode.INTAKE_CONE.speed);
        RollerMode.OUTTAKE_CONE.speed = SmartDashboard.getNumber("Outtake Cone Speed", RollerMode.OUTTAKE_CONE.speed);
        RollerMode.INTAKE_CUBE.speed = SmartDashboard.getNumber("Intake Cube Speed", RollerMode.INTAKE_CUBE.speed);
        RollerMode.OUTTAKE_CUBE.speed = SmartDashboard.getNumber("Outtake Cube Speed", RollerMode.OUTTAKE_CUBE.speed);
        RollerMode.INTAKE_CONE.time = SmartDashboard.getNumber("Intake Cone Time", RollerMode.INTAKE_CONE.time);
        RollerMode.OUTTAKE_CONE.time = SmartDashboard.getNumber("Outtake Cone Time", RollerMode.OUTTAKE_CONE.time);
        RollerMode.INTAKE_CUBE.time = SmartDashboard.getNumber("Intake Cube Time", RollerMode.INTAKE_CUBE.time);
        RollerMode.OUTTAKE_CUBE.time = SmartDashboard.getNumber("Outtake Cube Time", RollerMode.OUTTAKE_CUBE.time);

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
        for(int i = 0; i < ledBuffer.getLength(); i++)
            ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        led.setData(ledBuffer);
    }

    public boolean hasGamePiece() {
        return getGamePieceDistanceIn() < gamePieceDetectDistanceIn;
    }

    public double getGamePieceDistanceIn() {
        return Units.metersToInches((distSensor.getRange() - 16)/1000); // TODO: Remove magic numbers (waiting on @ProfessorAtomicManiac to get back to me)
    }

    public static class RollerMode {
        public static RollerMode INTAKE_CONE = new RollerMode(-0.5, .5, true);
        public static RollerMode INTAKE_CUBE = new RollerMode(0.3, .25, true);
        public static RollerMode OUTTAKE_CONE = new RollerMode(0.5, .5, false);
        public static RollerMode OUTTAKE_CUBE = new RollerMode(-0.5, .5, false);
        public double speed, time;
        public boolean intake;

        /**
         * @param speed A number between -1 and 1
         * @param time Amount of time in seconds to keep the motor running after
         * distance sensor has detected an object
         * @param intake Whether the roller is outtaking or intaking
         */
        public RollerMode(double speed, double time, boolean intake) {
            this.speed = speed;
            this.time = time;
            this.intake = intake;
        }
    }
}
