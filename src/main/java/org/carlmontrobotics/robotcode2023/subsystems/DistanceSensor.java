// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.subsystems;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceSensor extends SubsystemBase {

  /* constructor parameter: sensorID
  Methods:
  getRange() (gets distance)
  setRangingMode(rangingMode, sampleTime)
  getAmbientLightLevel()
  getStatus()




  */

  private TimeOfFlight distSensor = new TimeOfFlight(10);
  double dist;
  /** Creates a new DistanceSensor. */
  public DistanceSensor() {
    SmartDashboard.putNumber("distance", distSensor.getRange());
    SmartDashboard.putNumber("light", distSensor.getAmbientLightLevel());
    SmartDashboard.putString("Status", distSensor.getStatus().toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double dist = Units.metersToInches(distSensor.getRange()/1000);
    SmartDashboard.putNumber("distance", dist);
    SmartDashboard.putNumber("light", distSensor.getAmbientLightLevel());
    SmartDashboard.putString("Status", distSensor.getStatus().toString());
  }

  public int getErrorType() {
    // We need the type of offset, if any:
    // 0 means no adjustment, 1 means we need to move the robot left, and
    // 2 means we need to move the robot right. 

    if (dist < 7) return 1;
    else if (dist > 19) return 2;
    else return 0;
  }

  public double getDistance() {
    return dist;
  }
}