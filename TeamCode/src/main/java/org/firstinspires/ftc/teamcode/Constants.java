package org.firstinspires.ftc.teamcode;

public final class Constants {
    // Target voltage for voltage compensation
    public static final double kNominalVoltage = 12.0;

    // Multiplied by each term before assigning to the controller
    public static final double kPositionPIDFactor = 1.0 / 10000.0;
    public static final double kVelocityPIDFactor = 1.0 / 1000.0;

    public static final class Drive {
        public static final String[] kMotorNames = new String[]{
                "leftFront",
                "leftRear",
                "rightFront",
                "rightRear"
        };
        public static final double kDriveP = 5;
    }

    public static final class Vision {
        public static final double kTagsizeMeters = 0.0475;
    }

    public static final class Arm {
        public static final String kSlideName = "slide";

        public static final double kSlideP = 15;
        public static final double kSlideTolerance = 0.05;
        public static final double kSlideMaxVelocity = 1;
        public static final double kSlideG = 0.2; //THIS MIGHT BE AN ISSUE

        public static final double kSlideTop = 3925;
        public static final double kSlideMiddle = 2900;
        public static final double kSlideShort = 1800;
        public static final double kSlideCarry = 250;
        public static final double kSlideBottom = 11;

    }

    public static final class Scoop {
        public static final String kClawName = "claw";
        public static final double kOut = 0.65;
        public static final double kIn = 1;
    }
}
