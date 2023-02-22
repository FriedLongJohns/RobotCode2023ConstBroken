package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class Arm extends SubsystemBase
{
  public CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Arm.port, TemperatureLimit.NEO);
  public RelativeEncoder motorLencoder = motor.getEncoder();

  public double encoderErrorTolerance = .2;

  private double kS = .067766; //volts | base speed
  private double kG = .0075982; //volts | gravity... something
  private double kV = .019762; //volts*secs/rad | extra velocity
  private double kA = .00039212; //volts*secs^2/rad | vacceleration
  /// these are all units ^ , actual arm speed is determined by values in .calculate
  private double Kp = 8.0781;
  private double Kd = 3.0113;
  private double Ki = 0.0;
  
  private double setpoint = 2.2;
  private double FFvelocity = .3;
  private double FFaccel = .3;
  private ArmFeedforward armFeed = new ArmFeedforward(kS, kG, kV, kA);
  
  /* 
  public void ArmWithFeedforwardPID(double VelocitySetpoint) {
    
    
    motor.setVoltage(ArmFeedforward.calculate(VelocitySetpoint)
        + pid.calculate(motorLencoder.getVelocity(),VelocitySetpoint));
  } */
  
  // private PIDController pid = new PIDController(Kp, Ki, Kd);
  
  public double goalPos;
  private double Radians = motorLencoder.getPosition();

  public enum ArmPreset {
    INTAKE(0.31), MID(-1.74), HIGH(-1.83);
    
    public double value; //not static so SmartDashboard can touch [IMPORTANT TO KNOW!]
    ArmPreset(double value) {
      this.value = value;
    }
    public ArmPreset next() {
      switch (this) {
        case INTAKE: return MID;
        case MID: return HIGH;
        case HIGH: return INTAKE;
      }
      return null;
    }
    public ArmPreset prev(){
      switch (this) {
        case INTAKE: return HIGH;
        case MID: return INTAKE;
        case HIGH: return MID;
      }
      return null;
    }
    
  }

  public Arm(){
    motorLencoder.setPositionConversionFactor(2*Math.PI/100);
    motorLencoder.setPosition(0.0);
    SmartDashboard.putNumber("FF: Velocity", FFvelocity);
    SmartDashboard.putNumber("FF: Acceleration", FFaccel);
    SmartDashboard.putNumber("GoalPosition", goalPos);
    SmartDashboard.putString("Debooog", "No.");
  }

  @Override
  public void periodic() {
    FFvelocity = SmartDashboard.getNumber("FF: Velocity", FFvelocity);
    FFaccel = SmartDashboard.getNumber("FF: Acceleration", FFaccel);
    goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);
    SmartDashboard.putNumber("ArmLencoderPos", motorLencoder.getPosition());
    // pid.setTolerance(2.5,10);
    // pid.atSetpoint();

    
    // motor.set(pid.calculate( motorLencoder.getPosition(), setpoint));
    double difference = Math.abs(goalPos - motorLencoder.getPosition());//RADIANS
    if (difference > encoderErrorTolerance){//even PID needs an acceptable error sometimes
      //assuming calculate() is some sort of PID-esque thing
      motor.set(armFeed.calculate(goalPos + Math.PI/2, FFvelocity, FFaccel));
    } else {
      motor.set(0);
    }
    SmartDashboard.putNumber("WTF DOES THIS RETURN", armFeed.calculate(goalPos + Math.PI/2, FFvelocity, FFaccel));
    
    
  }

  //Snaps raw encoder pos to one of our cycle positions
  public ArmPreset snappedArmPos(){
    double encoderPos = motorLencoder.getPosition();
    
    for(ArmPreset check : ArmPreset.values()){
      double lowdist = (check.value - check.prev().value) / 2;
      double hidist = (check.next().value - check.value) / 2; // get the halfway points between each position and it's neighbors
      if (check.value - lowdist < encoderPos && encoderPos < check.value + hidist){
        //seperate high and low instead of ABS because maybe difference isn't constant between each position of arm
          //and yes it still works for lowest and highest value
          return check;
      }
    }
    //help something went wrong
    return null;
  }

  public ArmPreset closeSnappedArmPos() {//more precise snapping
    double encoderPos = motorLencoder.getPosition();
    
    for(ArmPreset check : ArmPreset.values()){
        if (Math.abs(check.value - encoderPos) > encoderErrorTolerance) {//maybe will break if cone/cube values are close, but if they are close then lower error or only use one enum
            return check;
        }
    }
    //help something went wrong
    return null;
  }

  public void cycleUp(){ 
    // because most people won't remember/want to do this long function chain
    SmartDashboard.putString("Debooog", "CycleUp");
    SmartDashboard.putNumber("GoalPosition", 
    closeSnappedArmPos() != null ? closeSnappedArmPos().next().value : snappedArmPos().next().value
    );
    // if closeSnappedArmPos is working, swap based on it - otherwise use less accurate snapping
  }
  
  public void cycleDown(){
    SmartDashboard.putString("Debooog", "CycleDown");
    SmartDashboard.putNumber("GoalPosition", 
    closeSnappedArmPos() != null ? closeSnappedArmPos().prev().value : snappedArmPos().prev().value
    );
  }
  
  public void setPreset(ArmPreset preset){
    SmartDashboard.putNumber("GoalPosition", preset.value);
  }
}

