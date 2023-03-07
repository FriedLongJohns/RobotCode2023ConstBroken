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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;



public class Arm extends SubsystemBase {

  //actual arm
//  private CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Arm.port, TemperatureLimit.NEO);
//  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  private double gearRatio = 225;
  
  public CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Arm.port, TemperatureLimit.NEO);
  public SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
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
  
  private double hiClamp = -Math.PI*.5; //TODO GET NUMBERS
  private double loClamp = -Math.PI*1.4;
  
  public final DoubleConsumer setGoalPos = (pos) -> {goalPos = MathUtil.clamp(pos, loClamp, hiClamp);};
  public final DoubleSupplier getGoalPos = () -> {return goalPos;};
  private double EncoderPos = encoder.getZeroOffset();

  public enum ArmPreset {
    INTAKE(0.31), MID(-1.74), HIGH(-1.83);
    
    public double value; //not static so SmartDashboard can touch [IMPORTANT TO KNOW!]
    ArmPreset(double value) {
      this.value = value;
    }
  }

  public Arm() {
    /*
    encoder.setPositionConversionFactor(1/gearRatio);
    encoder.getZeroOffset();
    pid.setTolerance(2.5,10);
    */
    
    //SmartDashboard.putNumber("GoalPosition", goalPos);

    SmartDashboard.putNumber("Motor Voltage", 0);
    encoder.setPositionConversionFactor(1 / gearRatio * 2 * Math.PI);
    motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Pos", encoder.getPosition());
    double speed = SmartDashboard.getNumber("Motor Voltage", 0);
    motor.set(speed);
    /*
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
     double currentPos = encoder.getZeroOffset();
    
    goalPos = MathUtil.clamp(goalPos, loClamp, hiClamp);
      
      motor.setVoltage(armFeed.calculate(currentPos, 0, 0)
         + pid.calculate(currentPos, goalPos));
         
         goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);
         SmartDashboard.getNumber("EncoderPos", currentPos);
         */
  }
  /*
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("FF: Velocity", () -> FFvelocity, x -> this.FFvelocity = x);
    builder.addDoubleProperty("FF: Accel",    () -> FFaccel,    x -> this.FFaccel = x);
    builder.addDoubleProperty("goalPos",      () -> goalPos,    x -> this.goalPos = x);
    builder.addDoubleProperty("EncoderPos",   () -> EncoderPos, x -> this.EncoderPos = x);
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
  */
}