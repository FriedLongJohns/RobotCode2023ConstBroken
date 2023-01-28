package org.carlmontrobotics.robotcode2023.subsystems;

import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.*;

import java.util.Arrays;
import java.util.function.Supplier;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.lib199.path.SwerveDriveInterface;
import org.carlmontrobotics.lib199.swerve.SwerveModule;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase implements SwerveDriveInterface {
    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP); // Also try kUSB and kUSB2

    private SwerveDriveKinematics kinematics = null;
    private SwerveDriveOdometry odometry = null;
    private SwerveModule modules[];
    private boolean fieldOriented = true;
    private double fieldOffset = 0;

    public final float initPitch;
    public final float initRoll;

    public Drivetrain() {
        // Calibrate Gyro
        {
            gyro.calibrate();
            double initTimestamp = Timer.getFPGATimestamp();
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
            System.out.println("Magnetometer is calibrated: " + gyro.isMagnetometerCalibrated());
        }

        // Setup Kinematics
        {
            // Define the corners of the robot relative to the center of the robot using
            // Translation2d objects.
            // Positive x-values represent moving toward the front of the robot whereas
            // positive y-values represent moving toward the left of the robot.
            Translation2d locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
            Translation2d locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
            Translation2d locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
            Translation2d locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

            kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
        }

        // Initialize modules
        {
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
        }

        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions());
    }

    @Override
    public void periodic() {
        for (SwerveModule module : modules) module.periodic();

        // Update the odometry with current heading and encoder position
        odometry.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());
    }

    @Override
    public void setOdometry(SwerveDriveOdometry odometry) {
        this.odometry = odometry;
    }

    @Override
    public SwerveDriveOdometry getOdometry() {
        return odometry;
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public boolean getFieldOriented() {
        return fieldOriented;
    }

    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }

    public void resetFieldOrientation() {
        fieldOffset = gyro.getAngle();
    }

    public void resetOdometry() {
        odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
        gyro.reset();
    }

    public double getHeading() {
        double x = gyro.getAngle();
        if (fieldOriented) { //TODO: field oriented
            x -= fieldOffset;
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

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        for(SwerveModule module : modules) SendableRegistry.addChild(this, module);

        builder.addBooleanProperty("Magnetic Field Disturbance", gyro::isMagneticDisturbance, null);
        builder.addBooleanProperty("Gyro Calibrating", gyro::isCalibrating, null);
        builder.addBooleanProperty("Field Oriented", () -> fieldOriented, fieldOriented -> this.fieldOriented = fieldOriented);
        builder.addDoubleProperty("Odometry X", () -> odometry.getPoseMeters().getX(), null);
        builder.addDoubleProperty("Odometry Y", () -> odometry.getPoseMeters().getY(), null);
        builder.addDoubleProperty("Odometry Heading", () -> odometry.getPoseMeters().getRotation().getDegrees(), null);
        builder.addDoubleProperty("Robot Heading", () -> getHeading(), null);
        builder.addDoubleProperty("Raw Gyro Angle", gyro::getAngle, null);
        builder.addDoubleProperty("Pitch", gyro::getPitch, null);
        builder.addDoubleProperty("Roll", gyro::getRoll, null);
        builder.addDoubleProperty("Field Offset", () -> fieldOffset, fieldOffset -> this.fieldOffset = fieldOffset);
    }

}
