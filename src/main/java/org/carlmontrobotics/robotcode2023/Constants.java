// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import java.awt.Color;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Arm {

        //#region Subsystem Constants

        // Array Indexes (Just to make things easier to read)
        public static final int ARM = 0;
        public static final int WRIST = 1;
        public static final int CUBE = 0;
        public static final int CONE = 1;
        public static final int FRONT = 0;
        public static final int BACK = 1;

        // Feedforward
        // Arm, Wrist
        public static final double[] kS = {0.20642, .074798};
        public static final double[] kG = {0.6697, 0.36214};
        public static final double[] kV = {4.3735, 1.6743};
        public static final double[] kA = {0.24914, 0.032177};
        public static final double kG_WRIST = .36214; // (V)

        // PID
        // FIXME BOTH WRIST AND ARM NEED TO TEST PID (Wrist PID never tested)
        // Arm, Wrist
        public static double[] kP = {6.7868, 4.7391}; // 4.2736 for arm from sysid was tested and it worked fine (V / rad)
        public static double[] kI = {0, 0}; // (V / (rad * s) )
        public static double[] kD = {4.4327, 0.69517}; // 0 for arm from sysid was tested and it worked fine (V / (rad / s) )

        // Arm, Wrist
        public static double[] posToleranceRad = { .05, .05 }; // rad
        public static double[] velToleranceRadPSec = { 0.5, 0.5 }; // rad/s

        // rad, rad/s
        public static TrapezoidProfile.State[] goalState = { new TrapezoidProfile.State(-Math.PI / 2, 0), new TrapezoidProfile.State(0, 0) }; // rad
        public static double[] offsetRad = { 4.02, -0.7 + Math.PI / 2 }; // rad

        // needed to calculate feedforward values dynamically
        public static final double ARM_MASS_KG = Units.lbsToKilograms(8.1);
        public static final double ARM_LENGTH_METERS = Units.inchesToMeters(38.25);

        // Distance from the arm motor to the center of mass of the  arm
        public static final double COM_ARM_LENGTH_METERS = Units.inchesToMeters(13.23);
        public static final double ROLLER_MASS_KG = Units.lbsToKilograms(14.04);

        // distance of center of mass of roller to the WRIST motor
        public static final double COM_ROLLER_LENGTH_METERS = Units.inchesToMeters(9.47);
        public static final double g = 9.81;

        public static final double V_PER_NM = 0.01423;

        // TODO: Replace these values with Design's actual values
        public static final double ARM_LOWER_LIMIT_RAD = -3 * Math.PI / 2;
        public static final double ARM_UPPER_LIMIT_RAD = Math.PI / 2;
        public static final double ARM_DISCONTINUITY_RAD = (ARM_LOWER_LIMIT_RAD + ARM_UPPER_LIMIT_RAD) / 2 - Math.PI;
        public static final double WRIST_LOWER_LIMIT_RAD = -Math.PI;
        public static final double WRIST_UPPER_LIMIT_RAD = Math.PI;
        public static final double WRIST_DISCONTINUITY_RAD = (WRIST_LOWER_LIMIT_RAD + WRIST_UPPER_LIMIT_RAD) / 2 - Math.PI;

        // TODO: Determine actual max vel/accel
        public static double[] MAX_FF_VEL = {1, 1}; // rad / s
        public static double[] MAX_FF_ACCEL = {1, 1}; // rad / s^2
        public static TrapezoidProfile.Constraints armConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL[ARM], MAX_FF_ACCEL[ARM]);
        public static TrapezoidProfile.Constraints wristConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL[WRIST], MAX_FF_ACCEL[WRIST]);

        //#endregion

        //#region Ports

        public static final int armMotorPort = 17;
        public static final int wristMotorPort = 19;
        public static final boolean[] inverted = { true, false };
        public static final double rotationToRad = 2 * Math.PI;
        //#endregion

        //#region Command Constants

        public static final double wristStowPos = Units.degreesToRadians(135);

        //#endregion
    }

    public static final class GoalPos {

        //#region Goal Positions
        // if intake/outtake on back, the "negative" pos will be used
        // 0 = "Front", 1 = "Back" (i.e. front refers to the side with the battery)
        // 0 = CUBE, 1 = CONE
        // GoalPos(arm, wrist)
        
        // TODO : Get angles for front
        public static GoalPos[][] LOW = {
            { // front
                new GoalPos(Units.degreesToRadians(-44.46), Units.degreesToRadians(117.17)), // cube
                new GoalPos(Units.degreesToRadians(0), Units.degreesToRadians(-100.78))             // cone
            },
            { // back
                new GoalPos(Units.degreesToRadians(-80.2), Units.degreesToRadians(61.37)), 
                new GoalPos(Units.degreesToRadians(-90), Units.degreesToRadians(133.24))
            }
        };
        public static GoalPos[][] MID = {
            {
                new GoalPos(Units.degreesToRadians(-85.98), Units.degreesToRadians(132.01)), 
                new GoalPos(Units.degreesToRadians(-106.85), Units.degreesToRadians(65.47)) 
            },
            {
                new GoalPos(Units.degreesToRadians(15.24), Units.degreesToRadians(-109.64)), 
                new GoalPos(Units.degreesToRadians(-47.63), Units.degreesToRadians(141.7)) 
            }
        };
        public static GoalPos[][] HIGH = {
            {
                new GoalPos(Units.degreesToRadians(-102.02), Units.degreesToRadians(-149.079)), 
                new GoalPos(Units.degreesToRadians(-104.18), Units.degreesToRadians(54.24)) 
            },
            {
                new GoalPos(Units.degreesToRadians(20.76), Units.degreesToRadians(-52.06)), 
                new GoalPos(Units.degreesToRadians(-14.07), Units.degreesToRadians(58.12)) 
            }
        };
        // TODO: Get positions for STORED, SHELF, and SUBSTATION
        public static GoalPos[][] STORED = {
            {
                new GoalPos(Units.degreesToRadians(-90), 0), 
                new GoalPos(40, -1.8861790155)
            },
            {
                new GoalPos(Units.degreesToRadians(-90), 0), 
                new GoalPos(40, -1.8861790155)
            }
        };
        public static GoalPos[][] SHELF = {
            {
                new GoalPos(Units.degreesToRadians(-90), 0), 
                new GoalPos(0, 0)
            },
            {
                new GoalPos(Units.degreesToRadians(-90), 0), 
                new GoalPos(0, 0)
            }
        };
        public static GoalPos[][] SUBSTATION = {
            {
                new GoalPos(Units.degreesToRadians(-90), 0), 
                new GoalPos(0, 0)
            },
            {
                new GoalPos(Units.degreesToRadians(-90), 0), 
                new GoalPos(0, 0)
            }
        };
        public static GoalPos[][] INTAKE = {
            {
                new GoalPos(Units.degreesToRadians(-72.5), Units.degreesToRadians(73.37)),
                new GoalPos(Units.degreesToRadians(-66.6), Units.degreesToRadians(29.94)) 
            },
            {
                new GoalPos(Units.degreesToRadians(-72.5), Units.degreesToRadians(73.37)),
                new GoalPos(Units.degreesToRadians(-66.6), Units.degreesToRadians(29.94)) 
            }
        };
        public double armPos, wristPos;

        public GoalPos(double armPos, double wristPos) {
            this.armPos = armPos;
            this.wristPos = wristPos;
        }
        //#endregion
    }

    public static final class Roller {
        //#region Subsystem Constants

        public static final int ledLength = 85;
        public static final double ledDefaultColorRestoreTime = 3; // The time in seconds after picking up a game piece to restore the LED color to defaultColor
        public static final Color defaultColor = new Color(0, 0, 200);
        public static final Color pickupSuccessColor = new Color(0, 200, 0);
        public static final Color conePickupColor = new Color(150, 150, 0);
        public static final Color cubePickupColor = new Color(50, 0, 200);

        public static final double distSensorDepthMM = 16;
        public static final double gamePieceDetectDistanceIn = 20;

        //#endregion

        //#region Ports
        public static final int rollerPort = 18;
        public static final int ledPort = 0;

        //#endregion

        //#region Command Constants
        // TODO: Determine actual speeds/timings for roller
        public static class RollerMode {
            public static RollerMode INTAKE_CONE = new RollerMode(-0.5, .5, true);
            public static RollerMode INTAKE_CUBE = new RollerMode(0.3, .25, true);
            public static RollerMode OUTTAKE_CONE = new RollerMode(0.5, .5, false);
            public static RollerMode OUTTAKE_CUBE = new RollerMode(-0.5, .5, false);
            public double speed, time;
            public boolean intake;
    
            /**
             * @param speed  A number between -1 and 1
             * @param time   Amount of time in seconds to keep the motor running after
             *               distance sensor has detected an object
             * @param intake Whether the roller is outtaking or intaking
             */
            public RollerMode(double speed, double time, boolean intake) {
                this.speed = speed;
                this.time = time;
                this.intake = intake;
            }
        }

        //#endregion
    }

    public static final class OI {
        public static final double JOY_THRESH = 0.01;
        public static final double MIN_AXIS_TRIGGER_VALUE = 0.25;
        public static final class Driver {
            public static final int port = 0;

        }

        public static final class Manipulator {
            public static final int port = 1;

            public static final int toggleCubeButton = Button.kLeftBumper.value;
            public static final int toggleFrontButton = Button.kRightBumper.value;

            public static final int storePosButton = Button.kA.value;
            public static final int lowPosButton = Button.kX.value;
            public static final int midPosButton = Button.kY.value;
            public static final int highPosButton = Button.kB.value;

            public static final int shelfPickupPOV = 0;
            public static final int intakeConePOV = 90;
            public static final int substationPickupPOV = 180;
            public static final int intakeCubePOV = 270;

            public static final Axis rollerIntakeCubeButton = Axis.kRightTrigger;
            public static final Axis rollerIntakeConeButton = Axis.kLeftTrigger;
        }
    }
}
