package org.firstinspires.ftc.teamcode;

public final class Constants {
    // Target voltage for voltage compensation
    public static final double K_NOMINAL_VOLTAGE = 12.0;

    // Multiplied by each term before assigning to the controller
    public static final double K_POSITION_PID_FACTOR = 1.0 / 10000.0;
    public static final double K_VELOCITY_PID_FACTOR = 1.0 / 1000.0;

    public static final class Drive {
        public static final String[] K_MOTOR_NAMES = new String[]{
                "leftFront",
                "leftRear",
                "rightFront",
                "rightRear"
        };
        public static final double K_DRIVE_P = 5;
    }

    public static final class Vision {
        public static final double K_TAGSIZE_METERS = 0.039;
    }

    public static final class Arm {
        public static final double OFFSET = 0;
        public static final String K_SLIDE_NAME = "slide";

        public static final double K_SLIDE_P = 20; // 15 best value
        public static final double K_SLIDE_TOLERANCE = 0.05;
        public static final double K_SLIDE_MAX_VELOCITY = 1;
        public static final double K_SLIDE_G = 0.2; //THIS MIGHT BE AN ISSUE
                                                    //"Didn't turn out to be" - Aarav
        public static final double K_SLIDE_TOP = 3860 - OFFSET;
        public static final double K_SLIDE_MIDDLE = 2900 - OFFSET;
        public static final double K_SLIDE_SHORT = 1800 - OFFSET;
        public static final double K_SLIDE_CARRY = 250 - OFFSET;
        public static final double K_SLIDE_BOTTOM = 10 - OFFSET;

    }

    public static final class Scoop {
        public static final String K_CLAW_NAME = "claw";
        public static final double K_OUT = 0.35;
        public static final double K_IN = 1;
    }
}
