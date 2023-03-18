// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
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

        public static double[] goalPosRad = { -Math.PI / 2, 0 }; // rad
        public static double[] offsetRad = { 4.02, 3.50 + Math.PI / 2 }; // rad

        // needed to calculate feedforward values dynamically
        public static final double ARM_MASS_KG = Units.lbsToKilograms(6.57);
        public static final double ARM_LENGTH_METERS = Units.inchesToMeters(38.25);

        // Distance from the arm motor to the center of mass of the  arm
        public static final double COM_ARM_LENGTH_METERS = Units.inchesToMeters(13.23);
        public static final double ROLLER_MASS_KG = Units.lbsToKilograms(10.91);

        // distance of center of mass of roller to the WRIST motor
        public static final double COM_ROLLER_LENGTH_METERS = Units.inchesToMeters(9.47);
        public static final double g = 9.81;

        public static final double V_PER_NM = 0;

        // TODO: Replace these values with Design's actual values
        public static final double ARM_LOWER_LIMIT_RAD = -3 * Math.PI / 2;
        public static final double ARM_UPPER_LIMIT_RAD = Math.PI / 2;
        public static final double ARM_DISCONTINUITY_RAD = (ARM_LOWER_LIMIT_RAD + ARM_UPPER_LIMIT_RAD) / 2 - Math.PI;
        public static final double WRIST_LOWER_LIMIT_RAD = -Math.PI;
        public static final double WRIST_UPPER_LIMIT_RAD = Math.PI;
        public static final double WRIST_DISCONTINUITY_RAD = (WRIST_LOWER_LIMIT_RAD + WRIST_UPPER_LIMIT_RAD) / 2 - Math.PI;

        // TODO: Determine actual max vel/accel
        public static double[] MAX_FF_VEL = {0.1, 0}; // rad / s
        public static double[] MAX_FF_ACCEL = {0.11, 0}; // rad / s
        //#endregion

        //#region Motor Details

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
        // TODO: PUT IN CORRECT POSITIONS
        // These positions are for if the intake/outtake takes place on the front (battery) side of the robot
        // if intake/outtake on back, the "negative" pos will be used
        // 0 = CUBE, 1 = CONE
        // {Cone, Cube}
        // GoalPos(arm, wrist)
       
        public static GoalPos[] LOW = {
          new GoalPos(Units.degreesToRadians(-134.46), Units.degreesToRadians(98.33)), 
          new GoalPos(Units.degreesToRadians(-90), Units.degreesToRadians(-119.62))
        };
        public static GoalPos[] MID = {
          new GoalPos(Units.degreesToRadians(-175.98), Units.degreesToRadians(113.17)), 
          new GoalPos(Units.degreesToRadians(-196.85), Units.degreesToRadians(46.63)) 
        };
        public static GoalPos[] HIGH = {
          new GoalPos(Units.degreesToRadians(-190.02), Units.degreesToRadians(-167.919)), 
          new GoalPos(Units.degreesToRadians(-194.18), Units.degreesToRadians(35.4)) 
        };
        public static GoalPos[] STORED = {
          new GoalPos(Units.degreesToRadians(-90), 0), 
          new GoalPos(40, -1.8861790155)
        };
        public static GoalPos[] SHELF = {
          new GoalPos(Units.degreesToRadians(-90), 0), 
          new GoalPos(0, 0)
        };
        public static GoalPos[] SUBSTATION = {
          new GoalPos(Units.degreesToRadians(-90), 0), 
          new GoalPos(0, 0)
        };
        public static GoalPos[] INTAKE = {
          new GoalPos(Units.degreesToRadians(-72.5), Units.degreesToRadians(73.37)),
          new GoalPos(Units.degreesToRadians(-66.6), Units.degreesToRadians(29.94)) 
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
        public static final class Driver {
            public static final int port = 0;
        }
        public static final class Manipulator {
            public static final int port = 1;
            public static final int toggleCubeCone = Button.kLeftBumper.value;
            public static final int toggleFrontBack = Button.kRightBumper.value;
            public static final int store = Button.kA.value;
            public static final int low = Button.kX.value;
            public static final int mid = Button.kY.value;
            public static final int high = Button.kB.value;
            // TODO: Determine real ports for the following buttons. I couldn't find what buttons referred to the "4-pad"
            public static final int shelf = -1;
            public static final int intake = -1;
            public static final int substation = -1;

            public static final int cycleUp = 1;//FIXME use correct buttons
            public static final int cycleDown = 2;
        }
        public static final class Controller {
          public static final int port = 2;

          public static Joystick controller = new Joystick(port);

          public static int X;
          public static int A;
          public static int B;
          public static int Y;
          public static int LB;
          public static int RB;
          public static int LT;
          public static int RT;
          public static int BACK;
          public static int START;

          //TODO: mode button setting to teletop init
          static {
              if (controller.getName().equals("Logitech Dual Action")) {
                  // Buttons and triggers
                  X = 1;
                  A = 2;
                  B = 3;
                  Y = 4;
                  LB = 5;
                  RB = 6;
                  LT = 7;
                  RT = 8;
                  BACK = 9;
                  START = 10;
              } else {
                  // Buttons and triggers for xbox controller
                  X = 3;
                  A = 1;
                  B = 2;
                  Y = 4;
                  LB = 5;
                  RB = 6;
                  BACK = 7;
                  START = 8;
                  //todo get arrow buttons
              }
           }
        }
    }
}
