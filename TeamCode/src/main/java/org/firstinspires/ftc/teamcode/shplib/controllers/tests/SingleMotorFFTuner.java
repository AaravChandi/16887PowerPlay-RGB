package org.firstinspires.ftc.teamcode.shplib.controllers.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(group = "ff")
public class SingleMotorFFTuner extends LinearOpMode {
    public static String MOTOR_NAME = "motor";
    public static double DISTANCE = 10; //

    public static double MAX_VEL = 1; // rotations / second
    public static double MAX_ACCEL = 1; // rotations / second


    @Override
    public void runOpMode() {

    }
}
