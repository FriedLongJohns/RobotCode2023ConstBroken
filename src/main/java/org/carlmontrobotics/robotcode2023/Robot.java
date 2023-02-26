// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import com.pathplanner.lib.server.PathPlannerServer;

import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.lib199.MotorErrors;
import org.carlmontrobotics.lib199.sim.MockedSparkEncoder;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
  private Limelight lime;
  private Drivetrain dt = new Drivetrain(lime);

  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    if(!DriverStation.isFMSAttached()) PathPlannerServer.startServer(5811);
    robotContainer = new RobotContainer();
    dt.driveForward();
  }

  @Override
  public void simulationInit() {
    MockedSparkEncoder.setGearing(Constants.Drivetrain.driveFrontLeftPort, Constants.Drivetrain.driveGearing);
    MockedSparkEncoder.setGearing(Constants.Drivetrain.driveFrontLeftPort, Constants.Drivetrain.driveGearing);
    MockedSparkEncoder.setGearing(Constants.Drivetrain.driveFrontLeftPort, Constants.Drivetrain.driveGearing);
    MockedSparkEncoder.setGearing(Constants.Drivetrain.driveFrontLeftPort, Constants.Drivetrain.driveGearing);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    MotorErrors.printSparkMaxErrorMessages();
  }

  @Override
  public void disabledInit() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        e.printStackTrace();
        return;
      }

      robotContainer.drivetrain.coast();
    }).start();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    robotContainer.drivetrain.brake();
    robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.drivetrain.brake();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.drivetrain.brake();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
