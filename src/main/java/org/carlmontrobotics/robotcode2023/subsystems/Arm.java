package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  
  //FIXME WRIST NEEDS SYSID DONE
  // kS, kG, kV, kA
  private final double[] armFeedConsts = {.067766, 0, .019762, .00039212};
  private final double[] wristFeedConsts = {.05, .007, .02, .0004};
  // Arm kG will be dynamically updated, pass 0 to constructor (kG is final in ArmFeedForward)
  private double armKG = 0;

  private double[] FFvelocity = {0,0};
  private double[] FFaccel =    {0,0};
  
  private ArmFeedforward armFeed = new ArmFeedforward(armFeedConsts[0], armFeedConsts[1], armFeedConsts[2], armFeedConsts[3]);
  private ArmFeedforward wristFeed = new ArmFeedforward(wristFeedConsts[0], wristFeedConsts[1], wristFeedConsts[2], wristFeedConsts[3]);
  
  //PID
  //FIXME BOTH WRIST AND ARM NEED PID DONE
  // Arm, Wrist for each "column"
  private double[] kP = {0,        0};
  private double[] kI = {0,        0};
  private double[] kD = {0,        0}; 
  
  private double[] posTolerance = {.05, .05};
  private double[] velTolerance = {1,   1};
  
  private PIDController armpid = new PIDController(kP[0], kI[0], kD[0]);
  private PIDController wristpid = new PIDController(kP[1], kI[1], kD[1]);

  private double[] goalPos = {-Math.PI / 2, 0};
  
  private double initialArmOffset = -Math.PI / 2;

  // needed to calculate feedforward values dynamically
  private final double ARM_MASS = 0;
  private final double ARM_LENGTH = 0;
  // fraction representing the distance from the arm motor to the center of mass
  // over the length of the whole arm
  private final double COM_ARM_LENGTH = 0; 
  private final double ROLLER_MASS = 0;
  // distance of center of mass of roller to the WRIST motor
  private final double COM_ROLLER_LENGTH = 0;
  private final double g = 9.81;
  
  private final double TORQUE_TO_KG = 0;
  
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
    armKG = calculateKG();
    double[] currentPos = new double[] {armEncoder.getPosition(), wristEncoder.getPosition()};

    wristMotor.setVoltage(wristFeed.calculate(getRobotWristAngle(), 0, 0)
        + wristpid.calculate(currentPos[1], goalPos[1]));
    
    Rotation2d currentAngle = getCoM().getAngle();
    armMotor.setVoltage(armFeed.calculate(currentAngle.getRadians(), 0, 0)
    + armpid.calculate(currentPos[0], goalPos[0]) + armKG * currentAngle.getCos());
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    //builder.addDoubleProperty("ArmFFVelocity",() -> FFvelocity[0],       x -> this.FFvelocity[0] = x);
    //builder.addDoubleProperty("WisFFVelocity",() -> FFvelocity[0],       x -> this.FFvelocity[0] = x);
    //builder.addDoubleProperty("ArmFFAccel",   () -> FFaccel[0],          x -> this.FFaccel[0] = x);
    //builder.addDoubleProperty("WisFFAccel",   () -> FFaccel[0],          x -> this.FFaccel[0] = x);
    builder.addDoubleProperty("ArmPos",       () -> armEncoder.getPosition(),   null);
    builder.addDoubleProperty("WisPos",       () -> wristEncoder.getPosition(), null);
    builder.addDoubleProperty("RobotWisPos",  () -> getRobotWristAngle(), null);
    builder.addDoubleProperty("kP: Arm",      () -> kP[0],            x -> this.kP[0] = x);
    builder.addDoubleProperty("kP: Wis",      () -> kP[1],            x -> this.kP[1] = x);
    builder.addDoubleProperty("kI: Arm",      () -> kI[0],            x -> this.kI[0] = x);
    builder.addDoubleProperty("kI: Wis",      () -> kI[1],            x -> this.kI[1] = x);
    builder.addDoubleProperty("kD: Arm",      () -> kD[0],            x -> this.kD[0] = x);
    builder.addDoubleProperty("kD: Wis",      () -> kD[1],            x -> this.kD[1] = x);
  } 
  
  public double getRobotWristAngle() {
    return (armEncoder.getPosition() + Math.PI / 2) + wristEncoder.getPosition();
  }

  // distance from center of mass relative to joint holding arm
  public Translation2d getCoM() {
    Translation2d comOfArm = new Translation2d(ARM_MASS * COM_ARM_LENGTH, new Rotation2d(armEncoder.getPosition()));
    // not exactly center of mass of wrist, but "close enough"
    Translation2d comOfWrist = new Translation2d(ROLLER_MASS * ARM_LENGTH, new Rotation2d(armEncoder.getPosition()));
    Translation2d comOfRoller = new Translation2d(ROLLER_MASS * COM_ROLLER_LENGTH, new Rotation2d(wristEncoder.getPosition()));
    return new Translation2d((comOfArm.getX() + comOfWrist.getX() + comOfRoller.getX()) / (ARM_MASS + ROLLER_MASS), 
                             (comOfArm.getY() + comOfWrist.getY() + comOfRoller.getY()) / (ARM_MASS + ROLLER_MASS));
  }

  public double maxHoldingTorque() {
    return (ARM_MASS + ROLLER_MASS) * g * getCoM().getDistance(new Translation2d());
  }

  public double calculateKG() {
    return TORQUE_TO_KG * maxHoldingTorque();
  }
}