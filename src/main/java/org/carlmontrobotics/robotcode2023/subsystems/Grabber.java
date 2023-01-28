// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.commands.CycleGrabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {

  private CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.grabber_motor_port, TemperatureLimit.NEO);

  public Grabber() {
    SmartDashboard.putNumber("Motor Voltage", 0);
    SmartDashboard.putNumber("Max Current", 0);

    SmartDashboard.putBoolean("open/close", false);

    SmartDashboard.putNumber("Open Speed", 0.1);
    SmartDashboard.putNumber("Close speed", -1);

    motor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    motor.setSmartCurrentLimit((int) SmartDashboard.getNumber("Max Current", 0));
    motor.set(SmartDashboard.getNumber("Motor Voltage", 0));
    SmartDashboard.putNumber("Motor Position", motor.getEncoder().getPosition());

    if (SmartDashboard.getBoolean("open/close", false)) {
      new CycleGrabber(this).schedule();
    }
  }
}
