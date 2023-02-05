// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.subsystems;

import java.awt.Color;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.SparkVelocityPIDController;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {

    private final CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.ROLLER_PORT, TemperatureLimit.NEO_550);

    public static final double coneIntakeConeOuttakeSpeed = .1;
    public static final double coneOuttakeConeIntakeSpeed = -.1;

    private final AddressableLED led = new AddressableLED(8);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(84);
    private double maxCurrent = 100;
    private boolean motorInverted = false;

    public Roller() {
        led.setLength(ledBuffer.getLength());
        // led.setData(ledBuffer);
        setLedColor(new Color(0, 200, 0));
        led.start();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Roller Current", motor.getOutputCurrent());
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
        return motor.getOutputCurrent() > maxCurrent;
    }
}
