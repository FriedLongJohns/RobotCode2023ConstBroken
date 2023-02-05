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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {

    private final CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.ROLLER_PORT, TemperatureLimit.NEO_550);

    public static final double coneIntakeConeOuttakeSpeed = 60;
    public static final double coneOuttakeConeIntakeSpeed = 60;

    private final AddressableLED led = new AddressableLED(8);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(84);
    private final SparkVelocityPIDController pidController = new SparkVelocityPIDController("Roller Motor", motor, 0, 0, 0, 0.2317, 0.061774, 0, 10);
    private boolean motorInverted = false;

    public Roller() {
        led.setLength(ledBuffer.getLength());
        led.start();
        LiveWindow.enableTelemetry(pidController);
    }

    @Override
    public void periodic() {}

    public void setSpeed(double speed) {
        motor.getEncoder().setVelocityConversionFactor(Math.copySign(1, speed));
        boolean requestedInversion = speed < 0;
        if(requestedInversion != motorInverted) {
            motorInverted = requestedInversion;
            motor.setInverted(motorInverted);
        }
        pidController.setTargetSpeed(Math.abs(speed));
    }

    public void setLedColor(Color color) {
        for(int i = 0; i < ledBuffer.getLength(); i++)
            ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
    }

    public boolean hasGamePiece() {
        return !pidController.isAtTargetSpeed();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        SendableRegistry.addChild(this, pidController);
    }
}
