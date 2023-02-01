// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
 
  private CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.ROLLER_PORT, TemperatureLimit.NEO_550);
 
  public Roller() {
    SmartDashboard.putNumber("motor speed", 0);
    SmartDashboard.putNumber("current draw", 0);

    motor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    motor.set(SmartDashboard.getNumber("motor speed", 0));
    SmartDashboard.putNumber("output current (amps)", motor.getOutputCurrent());
  }
}
