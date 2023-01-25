// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors;
import org.carlmontrobotics.lib199.SparkVelocityPIDController;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
  private CANSparkMax motor = MotorControllerFactory.createSparkMax(6, TemperatureLimit.NEO);

  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    robotContainer = new RobotContainer();
    SmartDashboard.putNumber("Motor Voltage", 0);
    SmartDashboard.putNumber("Max Current", 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    MotorErrors.printSparkMaxErrorMessages();
    motor.setSmartCurrentLimit((int) SmartDashboard.getNumber("Max Current", 0));
    motor.set(SmartDashboard.getNumber("Motor Voltage", 0));

  }


  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
