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
import edu.wpi.first.wpilibj.DigitalInput;
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
    public double intakeConeSpeed = -0.5;
    public double outtakeConeSpeed = 0.5;
    public double intakeCubeSpeed = 0.3;
    public double outtakeCubeSpeed = -0.5;
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

        SmartDashboard.putNumber("Roller Voltage", 0);
        SmartDashboard.putNumber("Intake Cone Speed", intakeConeSpeed);
        SmartDashboard.putNumber("Outtake Cone Speed", outtakeConeSpeed);
        SmartDashboard.putNumber("Intake Cube Speed", intakeCubeSpeed);
        SmartDashboard.putNumber("Outtake Cube Speed", outtakeCubeSpeed);
        SmartDashboard.putNumber("Time", 0);
        led.start();
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putBoolean("Has Game Piece", hasGamePiece());
        //setSpeed(SmartDashboard.getNumber("Roller Voltage", 0));
        // LED Update
        intakeConeSpeed = SmartDashboard.getNumber("Intake Cone Speed", intakeConeSpeed);
        outtakeConeSpeed = SmartDashboard.getNumber("Outtake Cone Speed", outtakeConeSpeed);
        intakeCubeSpeed = SmartDashboard.getNumber("Intake Cube Speed", intakeCubeSpeed);
        outtakeCubeSpeed = SmartDashboard.getNumber("Outtake Cube Speed", outtakeCubeSpeed);
        time = SmartDashboard.getNumber("Time", 0);
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
        
        SmartDashboard.putNumber("dist", Units.metersToInches((distSensor.getRange() - 16)/1000));
        return Units.metersToInches((distSensor.getRange() - 16)/1000) < 20;

        //return !beambreak.get();
    }
    public double getTime() {
        return time;
    }
}
