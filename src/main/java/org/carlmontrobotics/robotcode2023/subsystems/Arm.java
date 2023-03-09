package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;



public class Arm extends SubsystemBase {
  
  public CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Arm.port, TemperatureLimit.NEO);
  //public SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  public RelativeEncoder encoder = motor.getEncoder();

  public double encoderErrorTolerance = .05;

  private double kS = .067766; //volts | base speed
  private double kG = .0075982; //volts | gravity... something
  private double kV = .019762; //volts*secs/rad | extra velocity
  private double kA = .00039212; //volts*secs^2/rad | vacceleration
  /// these are all units ^ , actual arm speed is determined by values in .calculate
  private double kP = 1.1;
  private double kI = 1.1; //will add real values
  private double kD = 1.1;

  private double FFvelocity = .01;
  private double FFaccel = .01;
  private ArmFeedforward armFeed = new ArmFeedforward(kS, kG, kV, kA);
  private PIDController pid = new PIDController(kP, kI, kD);
  
  private double goalPos;
  //private double EncoderPos = encoder.getZeroOffset();
  private double EncoderPos = encoder.getPosition();
    
  public double hiClamp = -Math.PI*.5; //TODO GET NUMBERS
  public double loClamp = -Math.PI*1.4;

  public enum ArmPreset {
    //radians
    //aproximate values
    CUBEGROUNDPICKUP(1.71254781475), CONEGROUNDPICKUP(2.29021388861), MOVEGAMEPIECE(0), 
    CUBEOUTPUTLOW(1.02603845343), CUBEOUTPUTMID(0.837535040237), CUBEOUTPUTHIGH(0.768312472123), CONEOUTPUTLOW(4.100940749846), CONEOUTPUTMID(1.13715693466), CONEOUTPUTHIGH(1.46030263853);
    
    public double value; //not static so SmartDashboard can touch [IMPORTANT TO KNOW!]
    ArmPreset(double value) {
      this.value = value;
    }
    public ArmPreset swapType(){
      switch (this){
        case CUBEGROUNDPICKUP:
          return CONEGROUNDPICKUP;
        case CONEGROUNDPICKUP:
          return CUBEGROUNDPICKUP;
        case CUBEOUTPUTLOW:
          return CONEOUTPUTLOW;
        case CUBEOUTPUTMID:
          return CONEOUTPUTMID;
        case CUBEOUTPUTHIGH:
          return CONEOUTPUTHIGH;
        case CONEOUTPUTLOW:
          return CONEOUTPUTLOW;
        case CONEOUTPUTMID:
          return CUBEOUTPUTLOW;
        case CONEOUTPUTHIGH:
          return CUBEOUTPUTHIGH;
        default:
          return this;
      }
    }
  }

  public Arm() {
    encoder.setPositionConversionFactor(1/15);
    //encoder.setZeroOffset(-Math.PI / 2);
    encoder.setPosition(-Math.PI / 2);
    pid.setTolerance(2.5,10);
    
    SmartDashboard.putNumber("GoalPosition", goalPos);
  }

  @Override
  public void periodic() {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
     //double currentPos = encoder.getZeroOffset();
     double currentPos = encoder.getPosition();
    
    goalPos = MathUtil.clamp(goalPos, loClamp, hiClamp);
      
      motor.setVoltage(armFeed.calculate(currentPos, 0, 0)
         + pid.calculate(currentPos, goalPos));                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
         
         goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);
         SmartDashboard.getNumber("EncoderPos", currentPos);
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("FF: Velocity", () -> FFvelocity, x -> this.FFvelocity = x);
    builder.addDoubleProperty("FF: Accel",    () -> FFaccel,    x -> this.FFaccel = x);
    builder.addDoubleProperty("goalPos",      () -> goalPos,    x -> this.goalPos = x);
    builder.addDoubleProperty("kP",           () -> kP,         x -> this.kP = x);
    builder.addDoubleProperty("kI",           () -> kI,         x -> this.kI = x);
    builder.addDoubleProperty("kD",           () -> kD,         x -> this.kD = x);
    builder.addDoubleProperty("kV",           () -> kV,         x -> this.kV = x);
    builder.addDoubleProperty("kG",           () -> kG,         x -> this.kG = x);
    builder.addDoubleProperty("kS",           () -> kS,         x -> this.kS = x);
    builder.addDoubleProperty("kA",           () -> kA,         x -> this.kA = x);
}
  
  public void setPreset(ArmPreset preset) {
   // SmartDashboard.putNumber("GoalPosition", preset.value);
  }
}
