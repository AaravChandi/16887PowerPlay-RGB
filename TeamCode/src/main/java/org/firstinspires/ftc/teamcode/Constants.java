package org.firstinspires.ftc.teamcode;

public final class Constants {
    // Target voltage for voltage compensation
    public static final double kNominalVoltage = 12.0;

    // Multiplied by each term before assigning to the controller
    public static final double kPositionPIDFactor = 1.0 / 100.0;
    public static final double kVelocityPIDFactor = 1.0 / 1000.0;

    public static final class Drive {
        public static final String[] kMotorNames = new String[]{
                "leftFront",
                "leftRear",
                "rightFront",
                "rightRear"
        };
    }

    public static final class Vision {
        public static final double kTagsizeMeters = 0.0475;
    }

    public static final class Arm {
        public static final String kSlideName = "slide";
        public static final String kActuatorName = "actuator";

        public static final double kSlideP = 25;
        public static final double kSlideTolerance = 0.2;
        public static final double kSlideMaxVelocity = 1;

        public static final double kActuatorP = 5;
        public static final double kActuatorTolerance = 50;

        public static final double kSlideTop = 10;
        public static final double kSlideMiddle = 5;
        public static final double kSlideBottom = 0;

        public static final double kActuatorTop = 2500;
        public static final double kActuatorMiddle = 1000;
        public static final double kActuatorBottom = 10;
    }

    public static final class Scoop {
        public static final String kScoopName = "scoop";

        public static final double kTop = 0.5;
        public static final double kMiddle = 0.9;
        public static final double kBottom = 1.0;
    }
}
