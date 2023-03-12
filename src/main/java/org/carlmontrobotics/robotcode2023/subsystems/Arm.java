package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.robotcode2023.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// All angles are with 0 representing the arm/wrist being parallel to the floor on the intake side of the robot
public class Arm extends SubsystemBase {

    private CANSparkMax armMotor = MotorControllerFactory.createSparkMax(Constants.Arm.port, MotorConfig.NEO);
    private CANSparkMax wristMotor = MotorControllerFactory.createSparkMax(Constants.Wrist.port, MotorConfig.NEO);

    private SparkMaxAbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private SparkMaxAbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    // FIXME WRIST NEEDS SYSID DONE
    // kS (V), kG (V) (Wrist Only), kV (V/(rad/s)), kA (V/(rad/s^2))
    private final double[] armFeedConsts = { .067766, .019762, .00039212 };
    private final double[] wristFeedConsts = { .074798, 0.36214, 1.6743, 0.032177 };

    private double[] ffVelocityRadPSec = { 0, 0 }; // rad/s
    private double[] ffAccelRadPSec2 = { 0, 0 }; // rad/s^2

    private SimpleMotorFeedforward armFeed = new SimpleMotorFeedforward(armFeedConsts[0], armFeedConsts[1], armFeedConsts[2]);
    private ArmFeedforward wristFeed = new ArmFeedforward(wristFeedConsts[0], wristFeedConsts[1], wristFeedConsts[2], wristFeedConsts[3]);

    // PID
    // FIXME BOTH WRIST AND ARM NEED PID DONE
    // Arm, Wrist
    private double[] kP = { 0, 0 }; // (V / rad)
    private double[] kI = { 0, 0 }; // (V / (rad * s) )
    private double[] kD = { 0, 0 }; // (V / (rad / s) )

    // Arm, Wrist
    private double[] posToleranceRad = { .05, .05 }; // rad
    private double[] velToleranceRadPSec = { 0.5, 0.5 }; // rad/s

    private PIDController armPID = new PIDController(kP[0], kI[0], kD[0]);
    private PIDController wristPID = new PIDController(kP[1], kI[1], kD[1]);

    private double[] goalPosRad = { -Math.PI / 2, 0 }; // rad
    private double[] offsetRad = { 2.08, 4.02 }; // rad

    // needed to calculate feedforward values dynamically
    private final double ARM_MASS_KG = Units.lbsToKilograms(6.57);
    private final double ARM_LENGTH_METERS = Units.inchesToMeters(38.25);

    // Distance from the arm motor to the center of mass of the  arm
    private final double COM_ARM_LENGTH_METERS = Units.inchesToMeters(13.23);
    private final double ROLLER_MASS_KG = Units.lbsToKilograms(10.91);

    // distance of center of mass of roller to the WRIST motor
    private final double COM_ROLLER_LENGTH_METERS = Units.inchesToMeters(9.47);
    private final double g = 9.81;

    private final double V_PER_NM = 0;

    public Arm() {
        armMotor.setInverted(true);

        //Convert absolute encoders from rotations to degrees
        armEncoder.setPositionConversionFactor(2 * Math.PI);
        wristEncoder.setPositionConversionFactor(2 * Math.PI);

        wristEncoder.setZeroOffset(offsetRad[0]);
        armEncoder.setZeroOffset(offsetRad[1]);

        armPID.setTolerance(posToleranceRad[0], velToleranceRadPSec[0]);
        wristPID.setTolerance(posToleranceRad[1], velToleranceRadPSec[1]);
    }

    @Override
    public void periodic() {
        armMotor.setVoltage(getKg() * getCoM().getAngle().getCos() + armFeed.calculate(0, 0) + armPID.calculate(armEncoder.getPosition(), goalPosRad[0]));
        wristMotor.setVoltage(wristFeed.calculate(getWristPos(), 0, 0) + wristPID.calculate(getWristPos(), goalPosRad[1]));
    }

    // distance from center of mass relative to joint holding arm
    public Translation2d getCoM() {
        Translation2d comOfArm = new Translation2d(COM_ARM_LENGTH_METERS, Rotation2d.fromRadians(getArmPos())).times(ARM_MASS_KG);
        Translation2d comOfRoller = new Translation2d(ARM_LENGTH_METERS, Rotation2d.fromRadians(getArmPos()))
            .plus(new Translation2d(COM_ROLLER_LENGTH_METERS, Rotation2d.fromRadians(getArmPos() + getWristPos()))).times(ROLLER_MASS_KG);
        return comOfArm.plus(comOfRoller).div(ARM_MASS_KG + ROLLER_MASS_KG);
    }

    public double maxHoldingTorqueNM() {
        return (ARM_MASS_KG + ROLLER_MASS_KG) * g * getCoM().getNorm();
    }

    public double getKg() {
        return V_PER_NM * maxHoldingTorqueNM();
    }

    public double getArmPos() {
        return armEncoder.getPosition();
    }

    /**
     * @return The position of the wrist relative to the ground
     */
    public double getWristPos() {
        return getArmPos() + wristEncoder.getPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("ArmFFVelocity",() -> ffVelocityRadPSec[0], x -> this.ffVelocityRadPSec[0] = x);
        builder.addDoubleProperty("WisFFVelocity",() -> ffVelocityRadPSec[0], x -> this.ffVelocityRadPSec[0] = x);
        builder.addDoubleProperty("ArmFFAccel", () -> ffAccelRadPSec2[0], x -> this.ffAccelRadPSec2[0] = x);
        builder.addDoubleProperty("WisFFAccel", () -> ffAccelRadPSec2[0], x -> this.ffAccelRadPSec2[0] = x);
        builder.addDoubleProperty("WristGoalPos", () -> goalPosRad[1], goal -> this.goalPosRad[1] = goal);
        builder.addDoubleProperty("ArmPos", () -> getArmPos(), null);
        builder.addDoubleProperty("WisPos", () -> wristEncoder.getPosition(), null);
        builder.addDoubleProperty("RobotWisPos", () -> getWristPos(), null);
        builder.addDoubleProperty("kP: Arm", () -> kP[0], x -> this.kP[0] = x);
        builder.addDoubleProperty("kP: Wis", () -> kP[1], x -> this.kP[1] = x);
        builder.addDoubleProperty("kI: Arm", () -> kI[0], x -> this.kI[0] = x);
        builder.addDoubleProperty("kI: Wis", () -> kI[1], x -> this.kI[1] = x);
        builder.addDoubleProperty("kD: Arm", () -> kD[0], x -> this.kD[0] = x);
        builder.addDoubleProperty("kD: Wis", () -> kD[1], x -> this.kD[1] = x);
    }

}