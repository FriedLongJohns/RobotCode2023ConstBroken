// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.subsystems;

import java.awt.Color;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import com.revrobotics.CANSparkMax;

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

    private final CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.ROLLER_PORT, TemperatureLimit.NEO_550);
    private final DigitalInput beambreak = new DigitalInput(9);
    public static final double coneIntakeConeOuttakeSpeed = -.1;
    public static final double coneOuttakeConeIntakeSpeed = .1;

    private final AddressableLED led = new AddressableLED(8);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(84);
    private boolean hadGamePiece = false;
    private Command resetColorCommand = new SequentialCommandGroup(
        new WaitCommand(5),
        new InstantCommand(() -> new Color(0, 0, 200), this))
    {
        public boolean runsWhenDisabled() {
            return true;
        };
    };

    public Roller() {
        led.setLength(ledBuffer.getLength());
        // led.setData(ledBuffer);
        setLedColor(new Color(0, 0, 200));
        led.start();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Roller Current", motor.getOutputCurrent());
        SmartDashboard.putBoolean("Roller Has Game Piece", hasGamePiece());

        // LED Update
        {
            boolean hasGamePiece = hasGamePiece();
            if (hasGamePiece && !hadGamePiece) {
                setLedColor(new Color(0, 200, 0));
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
        return !beambreak.get();
    }
}
