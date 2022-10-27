package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FindAprilTagCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RRTankDrive;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.SHPMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@Autonomous

public class TestingAuto extends BaseRobot {
    AprilTagDetection currentTag;
    RRMecanumDrive drive;
    Trajectory traj1;
    Trajectory traj2;
    Trajectory traj3;


    @Override
    public void init() {
        //
        super.init();
        String[] motorNames = new String[]{
                "leftFront",
                "leftRear",
                "rightFront",
                "rightRear"
        };
        drive = new RRMecanumDrive(hardwareMap);
        //To get the current tag
        //currentTag.get(0);

        Pose2d startPos = new Pose2d(10, 10, Math.toRadians(90));
        drive.setPoseEstimate(startPos);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();


        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(20, 20), Math.toRadians(90))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(startPos)
                .forward(10)
                .strafeRight(10)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(startPos)
                .forward(10)
                .strafeLeft(10)
                .build();


        //telemetry.addData("Current Tag: ", currentTag.get(0));

    }

    public void start() {
        super.start();

        CommandScheduler.getInstance().scheduleCommand(
                new FindAprilTagCommand(vision)
                        .then(new RunCommand(() -> {
                            if (vision.getTags().get(0).id == 7)
                                drive.followTrajectory(traj1);
                            else if (vision.getTags().get(0).id == 8)
                                drive.followTrajectory(traj2);
                            else
                                drive.followTrajectory(traj3);

                        }))
        );






                /*.then(new RunCommand(() -> {
//                            currentTag = vision.getTags().get(0);


                        }))*/

    }



    @Override
    public void loop() {
        super.loop();
        for (AprilTagDetection tag : vision.getTags()) {
            telemetry.addData("Tag ID: ", tag.id);
        }
    }
    /*
    @Override
    public void start() {
        super.start();
        return;

        //driving :)
        //TODO: Get real coordinates
        //TODO: Get visualizer working
        /*Pose2d startPos = new Pose2d(10, 10, Math.toRadians(90));
        drive.setPoseEstimate(startPos);

        Trajectory parkTraj1 = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(0, 10), Math.toRadians(90))
                .build();
        Trajectory parkTraj2 = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(10, 15), Math.toRadians(90))
                .build();
        Trajectory parkTraj3 = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(20, 10), Math.toRadians(90))
                .build();

        drive.followTrajectory(parkTraj1);
/*
        if(Integer.valueOf(currentTag.get(0).toString()) == 7){
            drive.followTrajectory(parkTraj1);
        }
        else if(Integer.valueOf(currentTag.get(0).toString()) == 8){
            drive.followTrajectory(parkTraj2);
        }
        else if(Integer.valueOf(currentTag.get(0).toString()) == 12){
            drive.followTrajectory(parkTraj3);
        }

         */

    }





//
//        Pose2d startPos = new Pose2d(10, 10, Math.toRadians(90));
//        drive.setPoseEstimate(startPos);
//
//        telemetry.addData(">", "Press Play to start op mode");
//        telemetry.update();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        Trajectory traj1 = drive.trajectoryBuilder(startPos)
//                .splineTo(new Vector2d(20, 20), Math.toRadians(90))
//                .build();
//        drive.followTrajectory(traj1);
//
//        // do something
//
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .strafeLeft(10)
//                .build();
//        drive.followTrajectory(traj2);
//
//        // do something else
//
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .back(10)
//                .build();
//        drive.followTrajectory(traj3);
//
//        // done
//
//        return;


