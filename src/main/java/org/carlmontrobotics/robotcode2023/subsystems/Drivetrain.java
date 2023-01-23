package org.carlmontrobotics.robotcode2023.subsystems;

import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.*;

import java.util.Arrays;
import java.util.function.Supplier;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.lib199.path.SwerveDriveInterface;
import org.carlmontrobotics.lib199.swerve.SwerveModule;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase implements SwerveDriveInterface {
    // compassOffset is magnetic north relative to the current heading
    private final double compassOffset;
    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP); // Also try kUSB and kUSB2

    private SwerveDriveKinematics kinematics = null;
    private SwerveDriveOdometry odometry = null;
    private SwerveModule modules[];
    private static final boolean isGyroReversed = true;
    private static boolean fieldOriented = true;
    private double initTimestamp = 0;
    public final float initPitch;
    public final float initRoll;

    public Drivetrain() {
        gyro.calibrate();
        initTimestamp = Timer.getFPGATimestamp();
        double currentTimestamp = initTimestamp;
        while (gyro.isCalibrating() && currentTimestamp - initTimestamp < 10) {
            currentTimestamp = Timer.getFPGATimestamp();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
                break;
            }
            System.out.println("Calibrating the gyro...");
        }
        gyro.reset();
        System.out.println("NavX-MXP firmware version: " + gyro.getFirmwareVersion());
        SmartDashboard.putBoolean("Magnetic Field Disturbance", gyro.isMagneticDisturbance());
        System.out.println("Magnetometer is calibrated: " + gyro.isMagnetometerCalibrated());
        compassOffset = gyro.getCompassHeading();
        // Define the corners of the robot relative to the center of the robot using
        // Translation2d objects.
        // Positive x-values represent moving toward the front of the robot whereas
        // positive y-values represent moving toward the left of the robot.
        Translation2d locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
        Translation2d locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
        Translation2d locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
        Translation2d locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

        kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
        initPitch = 0;
        initRoll = 0;
        Supplier<Float> pitchSupplier = () -> initPitch;
        Supplier<Float> rollSupplier = () -> initRoll;
        // initPitch = gyro.getPitch();
        // initRoll = gyro.getRoll();
        // Supplier<Float> pitchSupplier = () -> gyro.getPitch();
        // Supplier<Float> rollSupplier = () -> gyro.getRoll();
        SwerveModule moduleFL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FL,
                MotorControllerFactory.createSparkMax(driveFrontLeftPort, TemperatureLimit.NEO),
                MotorControllerFactory.createSparkMax(turnFrontLeftPort, TemperatureLimit.NEO),
                MotorControllerFactory.createCANCoder(canCoderPortFL), 0,
                pitchSupplier, rollSupplier);
        // Forward-Right
        SwerveModule moduleFR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FR,
                MotorControllerFactory.createSparkMax(driveFrontRightPort, TemperatureLimit.NEO),
                MotorControllerFactory.createSparkMax(turnFrontRightPort, TemperatureLimit.NEO),
                MotorControllerFactory.createCANCoder(canCoderPortFR), 1,
                pitchSupplier, rollSupplier);
        // Backward-Left
        SwerveModule moduleBL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BL,
                MotorControllerFactory.createSparkMax(driveBackLeftPort, TemperatureLimit.NEO),
                MotorControllerFactory.createSparkMax(turnBackLeftPort, TemperatureLimit.NEO),
                MotorControllerFactory.createCANCoder(canCoderPortBL), 2,
                pitchSupplier, rollSupplier);
        // Backward-Right
        SwerveModule moduleBR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BR,
                MotorControllerFactory.createSparkMax(driveBackRightPort, TemperatureLimit.NEO),
                MotorControllerFactory.createSparkMax(turnBackRightPort, TemperatureLimit.NEO),
                MotorControllerFactory.createCANCoder(canCoderPortBR), 3,
                pitchSupplier, rollSupplier);
        modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions());

        SmartDashboard.putBoolean("Teleop Face Direction of Travel", false);
        SmartDashboard.putBoolean("Field Oriented", true);
        fieldOriented = SmartDashboard.getBoolean("Field Oriented", true);
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) {
            modules[i].periodic();
            // Uncommenting the following line will contribute to loop overrun errors
            // modules[i].updateSmartDashboard();
        }

        // Update the odometry with current heading and encoder position
        odometry.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());

        // SmartDashboard.putNumber("Odometry X",
        // odometry.getPoseMeters().getTranslation().getX());
        // SmartDashboard.putNumber("Odometry Y",
        // odometry.getPoseMeters().getTranslation().getY());;
        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
        SmartDashboard.putNumber("Raw gyro angle", gyro.getAngle());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        fieldOriented = SmartDashboard.getBoolean("Field Oriented", true);
        // SmartDashboard.putNumber("Gyro Compass Heading", gyro.getCompassHeading());
        // SmartDashboard.putNumber("Compass Offset", compassOffset);
        // SmartDashboard.putBoolean("Current Magnetic Field Disturbance",
        // gyro.isMagneticDisturbance());
    }

    @Override
    public void setOdometry(SwerveDriveOdometry odometry) {
        this.odometry = odometry;
    }

    @Override
    public SwerveDriveOdometry getOdometry() {
        return odometry;
    }

    public void resetFieldOrientation() {
        SmartDashboard.putNumber("Field Offset from North (degrees)", gyro.getAngle());
    }

    public void resetOdometry() {
        odometry.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(), new Pose2d());
        gyro.reset();
    }

    public double getHeading() {
        double x = gyro.getAngle();
        if (fieldOriented) { //TODO: field oriented
            x -= SmartDashboard.getNumber("Field Offset from North (degrees)", 0);
        }
        return Math.IEEEremainder(x * (isGyroReversed ? -1.0 : 1.0), 360);
    }

    // Resets the gyro, so that the direction the robotic currently faces is
    // considered "forward"
    public void resetHeading() {
        gyro.reset();
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void drive(double forward, double strafe, double rotation) {
        drive(getSwerveStates(forward, strafe, rotation));
    }

    @Override
    public void drive(SwerveModuleState[] moduleStates) {
        int num = 0;
        for (SwerveModuleState state : moduleStates) {
            num++;
            SmartDashboard.putNumber("state_angle_" + num, state.angle.getDegrees());
            SmartDashboard.putNumber("state_speed_" + num, state.speedMetersPerSecond);
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);

        // Move the modules based on desired (normalized) speed, desired angle, max
        // speed, drive modifier, and whether or not to reverse turning.
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = SwerveModuleState.optimize(moduleStates[i],
                    Rotation2d.fromDegrees(modules[i].getModuleAngle()));
            modules[i].move(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getDegrees());
        }
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(Arrays.stream(modules).map(SwerveModule::getCurrentState)
            .toArray(SwerveModuleState[]::new));
    }

    /**
     * Constructs and returns a ChassisSpeeds objects using forward, strafe, and
     * rotation values.
     * 
     * @param forward  The desired forward speed, in m/s.
     * @param strafe   The desired strafe speed, in m/s.
     * @param rotation The desired rotation speed, in rad/s.
     * @return A ChassisSpeeds object.
     */
    private ChassisSpeeds getChassisSpeeds(double forward, double strafe, double rotation) {
        ChassisSpeeds speeds;
        if (fieldOriented) { //TODO: field oriented
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(getHeading()).rotateBy(new Rotation2d(Math.PI)));
        } else {
            speeds = new ChassisSpeeds(forward, strafe, rotation);
        }
        return speeds;
    }

    /**
     * Constructs and returns four SwerveModuleState objects, one for each side,
     * using forward, strafe, and rotation values.
     * 
     * @param forward  The desired forward speed, in m/s.
     * @param strafe   The desired strafe speed, in m/s.
     * @param rotation The desired rotation speed, in rad/s.
     * @return A SwerveModuleState array, one for each side of the drivetrain (FL,
     *         FR, etc.).
     */
    private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
        return kinematics.toSwerveModuleStates(getChassisSpeeds(forward, -strafe, rotation));
    }

    public void toggleMode() {
        for (SwerveModule module: modules)
            module.toggleMode();
    }

    public void brake() {
        for (SwerveModule module: modules)
            module.brake();
    }

    public void coast() {
        for (SwerveModule module: modules)
            module.coast();
    }

    @Override
    public double getMaxAccelMps2() {
        return autoMaxAccelMps2;
    }

    @Override
    public double getMaxSpeedMps() {
        return autoMaxSpeedMps;
    }

    @Override
    public double getHeadingDeg() {
        return getHeading();
    }

    @Override
    public void stop() {
        for(SwerveModule module: modules) module.move(0, 0);
    }

    @Override
    public double[][] getPIDConstants() {
        return new double[][] {
            xPIDController,
            yPIDController,
            thetaPIDController
        };
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getCurrentPosition).toArray(SwerveModulePosition[]::new);
    }

}
