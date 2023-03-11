package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax;

public class Arm extends SubsystemBase {

  //actual arm
  private CANSparkMax armMotor = MotorControllerFactory.createSparkMax(Constants.Arm.port, MotorConfig.NEO);
  private CANSparkMax wristMotor = MotorControllerFactory.createSparkMax(Constants.Wrist.port, MotorConfig.NEO);
  
  // Conversion factor in radians
  private SparkMaxAbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  
  private double armGearRatio = 1/225;
  private double wristGearRatio = 1/84;
  //READ ME: Values are stored in the style of [arm,wrist]
  //0 is arm
  //1 is wrist
  
  //FeedForward
  //FIXME WRIST NEEDS SYSID DONE
  private double[] kS = {.067766,   .05}; //volts | base speed
  private double[] kG = {.0075982,  .007}; //volts | gravity... something
  private double[] kV = {.019762,   .02}; //volts*secs/rad | extra velocity
  private double[] kA = {.00039212, .0004}; //volts*secs^2/rad | vacceleration

  private double[] FFvelocity = {0,0};
  private double[] FFaccel =    {0,0};
  
  private ArmFeedforward armFeed = new ArmFeedforward(kS[0], kG[0], kV[0], kA[0]);
  private ArmFeedforward wristFeed = new ArmFeedforward(kS[1], kG[1], kV[1], kA[1]);
  
  //PID
  //FIXME BOTH WRIST AND ARM NEED PID DONE
  private double[] kP = {0,        0};
  private double[] kI = {0,        0};
  private double[] kD = {0,        0}; 
  
  private double[] posTolerance = {.05, .05};
  private double[] velTolerance = {1,   1};
  
  private PIDController armpid = new PIDController(kP[0], kI[0], kD[0]);
  private PIDController wristpid = new PIDController(kP[1], kI[1], kD[1]);

  private double[] goalPos = {-Math.PI / 2, 0};
  
  private double initialArmOffset = -Math.PI / 2;
  
  public Arm() {
    // Conversion factor in radians
    armEncoder.setPositionConversionFactor(armGearRatio * 2 * Math.PI);
    wristEncoder.setPositionConversionFactor(wristGearRatio * 2 * Math.PI);
    
    armEncoder.setZeroOffset(initialArmOffset);
    
    armpid.setTolerance(posTolerance[0], velTolerance[0]);
    wristpid.setTolerance(posTolerance[1], velTolerance[1]);
  }

  @Override
  public void periodic() {
    double[] currentPos = new double[] {armEncoder.getPosition(), wristEncoder.getPosition()};

    wristMotor.setVoltage(wristFeed.calculate(getRobotWristAngle(), 0, 0)
        + wristpid.calculate(currentPos[1], goalPos[1]));
    //arm
    armMotor.setVoltage(armFeed.calculate(currentPos[0], 0, 0)
    + armpid.calculate(currentPos[0], goalPos[0]));
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("ArmFFVelocity",() -> FFvelocity[0],       x -> this.FFvelocity[0] = x);
    builder.addDoubleProperty("WisFFVelocity",() -> FFvelocity[0],       x -> this.FFvelocity[0] = x);
    builder.addDoubleProperty("ArmFFAccel",   () -> FFaccel[0],          x -> this.FFaccel[0] = x);
    builder.addDoubleProperty("WisFFAccel",   () -> FFaccel[0],          x -> this.FFaccel[0] = x);
    builder.addDoubleProperty("ArmPos",       () -> armEncoder.getPosition(),   null);
    builder.addDoubleProperty("WisPos",       () -> wristEncoder.getPosition(), null);
    builder.addDoubleProperty("RobotWisPos",  () -> getRobotWristAngle(), null);
    builder.addDoubleProperty("kP: Arm",      () -> kP[0],            x -> this.kP[0] = x);
    builder.addDoubleProperty("kP: Wis",      () -> kP[1],            x -> this.kP[1] = x);
    builder.addDoubleProperty("kI: Arm",      () -> kI[0],            x -> this.kI[0] = x);
    builder.addDoubleProperty("kI: Wis",      () -> kI[1],            x -> this.kI[1] = x);
    builder.addDoubleProperty("kD: Arm",      () -> kD[0],            x -> this.kD[0] = x);
    builder.addDoubleProperty("kD: Wis",      () -> kD[1],            x -> this.kD[1] = x);
    builder.addDoubleProperty("kV: Arm",      () -> kV[0],            x -> this.kV[0] = x);
    builder.addDoubleProperty("kV: Wis",      () -> kV[1],            x -> this.kV[1] = x);
    builder.addDoubleProperty("kG: Arm",      () -> kG[0],            x -> this.kG[0] = x);
    builder.addDoubleProperty("kG: Wis",      () -> kG[1],            x -> this.kG[1] = x);
    builder.addDoubleProperty("kS: Arm",      () -> kS[0],            x -> this.kS[0] = x);
    builder.addDoubleProperty("kS: Wis",      () -> kS[1],            x -> this.kS[1] = x);
    builder.addDoubleProperty("kA: Arm",      () -> kA[0],            x -> this.kA[0] = x);
    builder.addDoubleProperty("kA: Wis",      () -> kA[1],            x -> this.kA[1] = x);
  } 
  
  public double getRobotWristAngle() {
    return (armEncoder.getPosition() + Math.PI / 2) + wristEncoder.getPosition();
  }
}