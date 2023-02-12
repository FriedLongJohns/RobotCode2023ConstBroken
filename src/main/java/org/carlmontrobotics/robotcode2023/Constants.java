// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

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

    public static final class Arm{
        public static final int portL = 6;
        public static final int portR = 8;
    }

    public static final class OI {
        public static final class Driver {
            public static final int port = 1;
        }
        public static final class Manipulator {
            public static final int port = 0;
            public static final int cone = Button.kRightBumper.value;
            public static final int cube = Button.kLeftBumper.value;
            public static final int intake = Button.kA.value;
            
            public static final int cycleUp = Button.kX.value;
            public static final int cycleDown = Button.kB.value;
            public static final int swapItemType = Button.kY.value;
        }
    }

}
