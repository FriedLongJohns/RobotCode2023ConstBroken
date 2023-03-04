package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class Arm extends SubsystemBase
{
  public CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Arm.port, TemperatureLimit.NEO);
  public RelativeEncoder motorLencoder = motor.getEncoder();

  public double encoderErrorTolerance = .1;

  private double kS = .17764; //volts | base speed
  private double kG = 4.1181; //volts | gravity... something
  private double kV = 1.7912; //volts*secs/rad | extra velocity
  private double kA = .15225; //volts*secs^2/rad | vacceleration
  /// these are all units ^ , actual arm0.15225 speed is determined by values in .calculate
  private double kP = 7.2985;
  private double kD = 2.2943;
  private double kI = 0.0;
  
  private double FFvelocity = 6;
  private double FFaccel = 6;
  private ArmFeedforward armFeed = new ArmFeedforward(kS, kG, kV, kA);
  private PIDController pid = new PIDController(kP, kI, kD);
  
  public double goalPos;

  public enum ArmPreset {
    INTAKE(0), MID(1.5), HIGH(2);
    
    public double value; //not static so SmartDashboard can touch [IMPORTANT TO KNOW!]
    ArmPreset(double value) {
      this.value = value;
    }
    public String toString() {
      switch (this) {
        case INTAKE: return "INTAKE";
        case MID: return "MID";
        case HIGH: return "HIGH";
      }
      return "null";
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
    motorLencoder.setPosition(-Math.PI / 2);
    SmartDashboard.putNumber("FF: Velocity", FFvelocity);
    SmartDashboard.putNumber("FF: Acceleration", FFaccel);
    SmartDashboard.putNumber("GoalPosition", goalPos);
    SmartDashboard.putNumber("KG", kG);
    SmartDashboard.putString("Debooog", "No.");
    SmartDashboard.putString("ArmENUM", 
    snappedArmPos().toString()
    );
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    pid.setTolerance(2.5,10);
  }

  @Override
  public void periodic() {
    FFvelocity = SmartDashboard.getNumber("FF: Velocity", FFvelocity);
    FFaccel = SmartDashboard.getNumber("FF: Acceleration", FFaccel);
    goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);
    SmartDashboard.putNumber("ArmLencoderPos", motorLencoder.getPosition());
    SmartDashboard.putString("ArmENUM", 
    (closeSnappedArmPos() != null) ? closeSnappedArmPos().toString() : snappedArmPos().toString()
    );
    kP = SmartDashboard.getNumber("kP", kP);
    kI = SmartDashboard.getNumber("kI", kI);
    kD = SmartDashboard.getNumber("kD", kD);
    kG = SmartDashboard.getNumber("KG", kG);
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    double currentPos = motorLencoder.getPosition();
    //pid.atSetpoint();

    // motor.set(pid.calculate( motorLencoder.getPosition(), setpoint));
    //FIXME CLAMP LIMIT FOR PROTOTYPE ONLY
    goalPos = MathUtil.clamp(goalPos, -Math.PI*1.4, -Math.PI*.5);
    //                           -4.39,  -1.57
    
      motor.setVoltage(armFeed.calculate(currentPos, 0, 0)
         + pid.calculate(currentPos, goalPos));
         
     
    
    
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    //builder.addBooleanProperty("Magnetic Field Disturbance", gyro::isMagneticDisturbance, null);
    //builder.addDoubleProperty("Odometry X", () -> getPose().getX(), null);
    builder.addDoubleProperty("FF: Velocity", () -> FFvelocity, x -> this.FFvelocity = x);
    builder.addDoubleProperty("FF: Accel",    () -> FFaccel,    x -> this.FFaccel = x);

    builder.addDoubleProperty("goalPos",      () -> goalPos,    x -> this.goalPos = x);

    builder.addDoubleProperty("kP",           () -> kP,         x -> this.kP = x);
    builder.addDoubleProperty("kI",           () -> kI,         x -> this.kI = x);
    builder.addDoubleProperty("kD",           () -> kD,         x -> this.kD = x);
}
  //Snaps raw encoder pos to one of our cycle positions
  public ArmPreset snappedArmPos(){
    double encoderPos = motorLencoder.getPosition();
    double bestD = Double.MAX_VALUE;
    ArmPreset bestE = closeSnappedArmPos();
    for(ArmPreset check : ArmPreset.values()){
      double dist = Math.abs(check.value - encoderPos);
      if (dist < bestD){
        bestD = dist;
        bestE = check;
      }
    }
    return bestE;
  }

  public ArmPreset closeSnappedArmPos() {//more precise snapping
    double encoderPos = motorLencoder.getPosition();
    
    for(ArmPreset check : ArmPreset.values()){
        if (Math.abs(check.value - encoderPos) < encoderErrorTolerance) {
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
    // if closeSnappedArmPos is workIng, swap based on it - otherwise use less accurate snapping
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

