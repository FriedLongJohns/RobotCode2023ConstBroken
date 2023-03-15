package org.carlmontrobotics.robotcode2023.subsystems;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.*;

import org.carlmontrobotics.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.robotcode2023.Constants.GoalPos;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    // 0 = cube, 1 = cone (used ints to make array indexing easier)
    private int isCube = 0;
    private boolean isFront = true;

    public Arm() {
        armMotor.setInverted(true);

        //Convert absolute encoders from rotations to degrees
        armEncoder.setPositionConversionFactor(2 * Math.PI);
        wristEncoder.setPositionConversionFactor(2 * Math.PI);

        wristEncoder.setZeroOffset(offsetRad[ARM]);
        armEncoder.setZeroOffset(offsetRad[WRIST]);

        armPID.setTolerance(posToleranceRad[ARM], velToleranceRadPSec[ARM]);
        wristPID.setTolerance(posToleranceRad[WRIST], velToleranceRadPSec[WRIST]);

        //TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES TO MASTER IF THIS IS STILL HERE)
        SmartDashboard.putNumber("LowArmCube", GoalPos.LOW[0].armPos);
        SmartDashboard.putNumber("LowWristCube", GoalPos.LOW[0].wristPos);
        SmartDashboard.putNumber("MidArmCube", GoalPos.MID[0].armPos);
        SmartDashboard.putNumber("MidWristCube", GoalPos.MID[0].wristPos);
        SmartDashboard.putNumber("HighArmCube", GoalPos.HIGH[0].armPos);
        SmartDashboard.putNumber("HighWristCube", GoalPos.HIGH[0].wristPos);
        SmartDashboard.putNumber("StoredArmCube", GoalPos.STORED[0].armPos);
        SmartDashboard.putNumber("StoredWristCube", GoalPos.STORED[0].wristPos);
        SmartDashboard.putNumber("ShelfArmCube", GoalPos.SHELF[0].armPos);
        SmartDashboard.putNumber("ShelfWristCube", GoalPos.SHELF[0].wristPos);
        SmartDashboard.putNumber("SubstationArmCube", GoalPos.SUBSTATION[0].armPos);
        SmartDashboard.putNumber("SubstationWristCube", GoalPos.SUBSTATION[0].wristPos);
        SmartDashboard.putNumber("IntakeArmCube", GoalPos.INTAKE[0].armPos);
        SmartDashboard.putNumber("IntakeWristCube", GoalPos.INTAKE[0].wristPos);

        SmartDashboard.putNumber("LowArmCone", GoalPos.LOW[1].armPos);
        SmartDashboard.putNumber("LowWristCone", GoalPos.LOW[1].wristPos);
        SmartDashboard.putNumber("MidArmCone", GoalPos.MID[1].armPos);
        SmartDashboard.putNumber("MidWristCone", GoalPos.MID[1].wristPos);
        SmartDashboard.putNumber("HighArmCone", GoalPos.HIGH[1].armPos);
        SmartDashboard.putNumber("HighWristCone", GoalPos.HIGH[1].wristPos);
        SmartDashboard.putNumber("StoredArmCone", GoalPos.STORED[1].armPos);
        SmartDashboard.putNumber("StoredWristCone", GoalPos.STORED[1].wristPos);
        SmartDashboard.putNumber("ShelfArmCone", GoalPos.SHELF[1].armPos);
        SmartDashboard.putNumber("ShelfWristCone", GoalPos.SHELF[1].wristPos);
        SmartDashboard.putNumber("SubstationArmCone", GoalPos.SUBSTATION[1].armPos);
        SmartDashboard.putNumber("SubstationWristCone", GoalPos.SUBSTATION[1].wristPos);
        SmartDashboard.putNumber("IntakeArmCone", GoalPos.INTAKE[1].armPos);
        SmartDashboard.putNumber("IntakeWristCone", GoalPos.INTAKE[1].wristPos);

        SmartDashboard.putNumber("Max FF Vel", MAX_FF_VEL);
        SmartDashboard.putNumber("Max FF Accel", MAX_FF_ACCEL);
    }

    @Override
    public void periodic() {
        
        //TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES TO MASTER IF THIS IS STILL HERE)
        armPID.setP(kP[ARM]);
        armPID.setI(kI[ARM]);
        armPID.setD(kD[ARM]);
        wristPID.setP(kP[WRIST]);
        wristPID.setI(kI[WRIST]);
        wristPID.setD(kD[WRIST]);

        GoalPos.LOW[0].armPos = SmartDashboard.getNumber("LowArmCube", GoalPos.LOW[0].armPos);
        GoalPos.LOW[0].wristPos = SmartDashboard.getNumber("LowWristCube", GoalPos.LOW[0].wristPos);
        GoalPos.MID[0].armPos = SmartDashboard.getNumber("MidArmCube", GoalPos.MID[0].armPos);
        GoalPos.MID[0].wristPos = SmartDashboard.getNumber("MidWristCube", GoalPos.MID[0].wristPos);
        GoalPos.HIGH[0].armPos = SmartDashboard.getNumber("HighArmCube", GoalPos.HIGH[0].armPos);
        GoalPos.HIGH[0].wristPos = SmartDashboard.getNumber("HighWristCube", GoalPos.HIGH[0].wristPos);
        GoalPos.STORED[0].armPos = SmartDashboard.getNumber("StoredArmCube", GoalPos.STORED[0].armPos);
        GoalPos.STORED[0].wristPos = SmartDashboard.getNumber("StoredWristCube", GoalPos.STORED[0].wristPos);
        GoalPos.SHELF[0].armPos = SmartDashboard.getNumber("ShelfArmCube", GoalPos.SHELF[0].armPos);
        GoalPos.SHELF[0].wristPos = SmartDashboard.getNumber("ShelfWristCube", GoalPos.SHELF[0].wristPos);
        GoalPos.SUBSTATION[0].armPos = SmartDashboard.getNumber("SubstationArmCube", GoalPos.SUBSTATION[0].armPos);
        GoalPos.SUBSTATION[0].wristPos = SmartDashboard.getNumber("SubstationWristCube", GoalPos.SUBSTATION[0].wristPos);
        GoalPos.INTAKE[0].armPos = SmartDashboard.getNumber("IntakeArmCube", GoalPos.INTAKE[0].armPos);
        GoalPos.INTAKE[0].wristPos = SmartDashboard.getNumber("IntakeWristCube", GoalPos.INTAKE[0].wristPos);

        GoalPos.LOW[1].armPos = SmartDashboard.getNumber("LowArmCone", GoalPos.LOW[1].armPos);
        GoalPos.LOW[1].wristPos = SmartDashboard.getNumber("LowWristCone", GoalPos.LOW[1].wristPos);
        GoalPos.MID[1].armPos = SmartDashboard.getNumber("MidArmCone", GoalPos.MID[1].armPos);
        GoalPos.MID[1].wristPos = SmartDashboard.getNumber("MidWristCone", GoalPos.MID[1].wristPos);
        GoalPos.HIGH[1].armPos = SmartDashboard.getNumber("HighArmCone", GoalPos.HIGH[1].armPos);
        GoalPos.HIGH[1].wristPos = SmartDashboard.getNumber("HighWristCone", GoalPos.HIGH[1].wristPos);
        GoalPos.STORED[1].armPos = SmartDashboard.getNumber("StoredArmCone", GoalPos.STORED[1].armPos);
        GoalPos.STORED[1].wristPos = SmartDashboard.getNumber("StoredWristCone", GoalPos.STORED[1].wristPos);
        GoalPos.SHELF[1].armPos = SmartDashboard.getNumber("ShelfArmCone", GoalPos.SHELF[1].armPos);
        GoalPos.SHELF[1].wristPos = SmartDashboard.getNumber("ShelfWristCone", GoalPos.SHELF[1].wristPos);
        GoalPos.SUBSTATION[1].armPos = SmartDashboard.getNumber("SubstationArmCone", GoalPos.SUBSTATION[1].armPos);
        GoalPos.SUBSTATION[1].wristPos = SmartDashboard.getNumber("SubstationWristCone", GoalPos.SUBSTATION[1].wristPos);
        GoalPos.INTAKE[1].armPos = SmartDashboard.getNumber("IntakeArmCone", GoalPos.INTAKE[1].armPos);
        GoalPos.INTAKE[1].wristPos = SmartDashboard.getNumber("IntakeWristCone", GoalPos.INTAKE[1].wristPos);

        MAX_FF_VEL = SmartDashboard.getNumber("Max FF Vel", MAX_FF_VEL);
        MAX_FF_ACCEL = SmartDashboard.getNumber("Max FF Accel", MAX_FF_ACCEL);
    }

    public void setArmVoltage(double volts) {
        armMotor.setVoltage(volts);
    }
    public void setWristVoltage(double volts) {
        wristMotor.setVoltage(volts);
    }
    public double driveArm(double vel, double accel) {
        if (Math.abs(ARM_ANGLE_MAX - getArmPos()) < ARM_ANGLE_TOLERANCE)
            return getKg() * getCoM().getAngle().getCos() + armFeed.calculate(0, 0) + armPID.calculate(armEncoder.getPosition(), goalPosRad[ARM]);
        return getKg() * getCoM().getAngle().getCos() + armFeed.calculate(vel, accel) + armPID.calculate(armEncoder.getPosition(), goalPosRad[ARM]);
    }
    public double driveWrist(double vel, double accel) {
        if (Math.abs(WRIST_ANGLE_MAX - getWristPos()) < WRIST_ANGLE_TOLERANCE)
            return wristFeed.calculate(getWristPosRelativeToGround(), 0, 0) + wristPID.calculate(getWristPos(), goalPosRad[WRIST]);
        return wristFeed.calculate(getWristPosRelativeToGround(), vel, accel) + wristPID.calculate(getWristPos(), goalPosRad[WRIST]);
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
        return MathUtil.clamp(MathUtil.inputModulus(armEncoder.getPosition(), ARM_ANGLE_MIN, ARM_ANGLE_MAX), ARM_ANGLE_MIN + ARM_ANGLE_TOLERANCE, ARM_ANGLE_MAX - ARM_ANGLE_TOLERANCE);
    }

    public double getWristPos() {
        return MathUtil.clamp(MathUtil.inputModulus(wristEncoder.getPosition(), WRIST_ANGLE_MIN, WRIST_ANGLE_MAX), WRIST_ANGLE_MIN + WRIST_ANGLE_TOLERANCE, WRIST_ANGLE_MAX - WRIST_ANGLE_TOLERANCE);
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

    public boolean armAtSetpoint() {
        return armPID.atSetpoint();
    }

    public boolean wristAtSetpoint() {
        return wristPID.atSetpoint();
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

    public boolean isCubeMode() {
        return isCube == 0;
    }
    public boolean isFrontMode() {
        return isFront;
    }
    public void toggleCube() {
        isCube++;
        isCube %= 2;
    }
    public void toggleFront() {
        isFront = !isFront;
    }
    public double getArmGoal(GoalPos[] pos) {
        if (isFront) {
            return pos[isCube].armPos;
        } else {
            return Rotation2d.fromRadians(-Math.PI).minus(Rotation2d.fromRadians(pos[isCube].armPos)).getRadians();
        }
    }
    public double getWristGoal(GoalPos[] pos) {
        if (isFront) {
            return pos[isCube].wristPos;
        } else {
            return Rotation2d.fromRadians(pos[isCube].wristPos).unaryMinus().getRadians();
        }
    }
}