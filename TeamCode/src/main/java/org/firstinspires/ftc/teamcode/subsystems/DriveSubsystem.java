package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.SHPMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.hardware.sensors.SHPIMU;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class DriveSubsystem extends Subsystem {
//    private final RRMecanumDrive rr;
    private final SHPMecanumDrive drive;
    public final SHPIMU imu;
    private double bias = 1.0;

    public DriveSubsystem(HardwareMap hardwareMap) {
//        rr = new RRMecanumDrive(hardwareMap, Constants.Drive.kMotorNames);
        drive = new SHPMecanumDrive(hardwareMap, Constants.Drive.K_MOTOR_NAMES);

        for (int i = 0; i<4; i++)
            drive.motors[i].enablePositionPID(Constants.Drive.K_DRIVE_P);

        // Change AxesOrder and AxesSigns according to your hub orientation
        // Omit Axes arguments for standard orientation
        imu = new SHPIMU(hardwareMap, AxesOrder.ZYX, AxesSigns.PPN);
    }

    public void mecanum(double leftY, double leftX, double rightX) {
        Vector2d vector = new Vector2d(
                leftY,
                leftX
        ).rotated(-imu.getYaw()+PoseStorage.offset);


        drive.mecanum(bias*vector.getX(), bias*vector.getY(), bias*0.8*rightX); // field oriented
    }
    public void setDriveBias(double driveBias) {
        bias = Range.clip(driveBias, 0.4, 1.0);
    }

    public void normalmecanum(double leftY, double leftX, double rightX) {
        drive.mecanum(leftY, leftX, rightX); // robot oriented
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("heading: ", Math.toDegrees(imu.getYaw()));
        telemetry.addData("leftFront: ", drive.motors[0].getPosition(MotorUnit.TICKS));
        telemetry.addData("leftRear: ", drive.motors[1].getPosition(MotorUnit.TICKS));
        telemetry.addData("rightFront: ", drive.motors[2].getPosition(MotorUnit.TICKS));
        telemetry.addData("rightRear: ", drive.motors[3].getPosition(MotorUnit.TICKS));

    }

}
