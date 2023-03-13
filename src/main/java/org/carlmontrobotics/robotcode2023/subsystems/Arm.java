package org.carlmontrobotics.robotcode2023.subsystems;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.*;

import org.carlmontrobotics.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Arm angle is measured from horizontal on the intake side of the robot and bounded between -3π/2 and π/2
// Wrist angle is measured relative to the arm with 0 being parallel to the arm and bounded between -π and π
public class Arm extends SubsystemBase {

    // Array Indexes (Just to make things easier to read)
    private static final int ARM = 0;
    private static final int WRIST = 1;

    private final CANSparkMax armMotor = MotorControllerFactory.createSparkMax(armMotorPort, MotorConfig.NEO);
    private final CANSparkMax wristMotor = MotorControllerFactory.createSparkMax(wristMotorPort, MotorConfig.NEO);

    private final SparkMaxAbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private final SparkMaxAbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private final SimpleMotorFeedforward armFeed = new SimpleMotorFeedforward(kS[ARM], kV[ARM], kA[ARM]);
    private final ArmFeedforward wristFeed = new ArmFeedforward(kS[WRIST], kG_WRIST, kV[WRIST], kA[WRIST]);

    private final PIDController armPID = new PIDController(kP[ARM], kI[ARM], kD[ARM]);
    private final PIDController wristPID = new PIDController(kP[WRIST], kI[WRIST], kD[WRIST]);

    public Arm() {
        armMotor.setInverted(true);

        //Convert absolute encoders from rotations to degrees
        armEncoder.setPositionConversionFactor(2 * Math.PI);
        wristEncoder.setPositionConversionFactor(2 * Math.PI);

        wristEncoder.setZeroOffset(offsetRad[ARM]);
        armEncoder.setZeroOffset(offsetRad[WRIST]);

        armPID.setTolerance(posToleranceRad[ARM], velToleranceRadPSec[ARM]);
        wristPID.setTolerance(posToleranceRad[WRIST], velToleranceRadPSec[WRIST]);
    }

    @Override
    public void periodic() {
        armMotor.setVoltage(getKg() * getCoM().getAngle().getCos() + armFeed.calculate(0, 0) + armPID.calculate(armEncoder.getPosition(), goalPosRad[ARM]));
        wristMotor.setVoltage(wristFeed.calculate(getWristPosRelativeToGround(), 0, 0) + wristPID.calculate(getWristPos(), goalPosRad[WRIST]));

        //TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES TO MASTER IF THIS IS STILL HERE)
        armPID.setP(kP[ARM]);
        armPID.setI(kI[ARM]);
        armPID.setD(kD[ARM]);
        wristPID.setP(kP[WRIST]);
        wristPID.setI(kI[WRIST]);
        wristPID.setD(kD[WRIST]);
    }

    // distance from center of mass relative to joint holding arm
    public Translation2d getCoM() {
        Translation2d comOfArm = new Translation2d(COM_ARM_LENGTH_METERS, Rotation2d.fromRadians(getArmPos())).times(ARM_MASS_KG);
        Translation2d comOfRoller = new Translation2d(ARM_LENGTH_METERS, Rotation2d.fromRadians(getArmPos()))
            .plus(new Translation2d(COM_ROLLER_LENGTH_METERS, Rotation2d.fromRadians(getWristPosRelativeToGround()))).times(ROLLER_MASS_KG);
        return comOfArm.plus(comOfRoller).div(ARM_MASS_KG + ROLLER_MASS_KG);
    }

    public double maxHoldingTorqueNM() {
        return (ARM_MASS_KG + ROLLER_MASS_KG) * g * getCoM().getNorm();
    }

    public double getKg() {
        return V_PER_NM * maxHoldingTorqueNM();
    }

    public double getArmPos() {
        return MathUtil.inputModulus(armEncoder.getPosition(), ARM_ANGLE_MIN, ARM_ANGLE_MAX);
    }

    public double getWristPos() {
        return MathUtil.inputModulus(wristEncoder.getPosition(), WRIST_ANGLE_MIN, WRIST_ANGLE_MAX);
    }

    // Unbounded wrist position relative to ground
    public double getWristPosRelativeToGround() {
        return getArmPos() + wristEncoder.getPosition();
    }

    public void setArmTarget(double target) {
        goalPosRad[ARM] = MathUtil.inputModulus(target, ARM_ANGLE_MIN, ARM_ANGLE_MAX);
    }

    public void setWristTarget(double target) {
        goalPosRad[WRIST] = MathUtil.inputModulus(target, WRIST_ANGLE_MIN, WRIST_ANGLE_MAX);
    }

    public void setArmWristTarget(double targetArm, double targetWrist) {
        setArmTarget(targetArm);
        setWristTarget(targetWrist);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("WristGoalPos", () -> goalPosRad[1], goal -> goalPosRad[1] = goal);
        builder.addDoubleProperty("ArmPos", () -> getArmPos(), null);
        builder.addDoubleProperty("WisPos", () -> getWristPos(), null);
        builder.addDoubleProperty("RobotWisPos", () -> getWristPosRelativeToGround(), null);
        builder.addDoubleProperty("kP: Arm", () -> kP[ARM], x -> kP[ARM] = x);
        builder.addDoubleProperty("kP: Wis", () -> kP[WRIST], x -> kP[WRIST] = x);
        builder.addDoubleProperty("kI: Arm", () -> kI[ARM], x -> kI[ARM] = x);
        builder.addDoubleProperty("kI: Wis", () -> kI[WRIST], x -> kI[WRIST] = x);
        builder.addDoubleProperty("kD: Arm", () -> kD[ARM], x -> kD[ARM] = x);
        builder.addDoubleProperty("kD: Wis", () -> kD[WRIST], x -> kD[WRIST] = x);
    }

}