package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.FindAprilTagCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RRTankDrive;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@Autonomous

public class TestingAuto extends BaseRobot {
    AprilTagDetection currentTag;
    RRMecanumDrive drive;

    /*public void runOpMode(){
        drive = new RRMecanumDrive(hardwareMap);

        waitForStart();
        new FindAprilTagCommand(vision);

        if (isStopRequested()) return;

        Pose2d startPos = new Pose2d(10, 10, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        Trajectory parkTraj1 = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(10, 20), Math.toRadians(0))
                .build();
        drive.followTrajectory(parkTraj1);
        return;


    } */

    @Override
    public void init() {
        //
        super.init();
        drive = new RRMecanumDrive(hardwareMap);
        //To get the current tag
        //currentTag.get(0);



        //telemetry.addData("Current Tag: ", currentTag.get(0));

    }

    public void start() {
        super.start();

        Pose2d startPos = new Pose2d(10, 10, Math.toRadians(90));
        drive.setPoseEstimate(startPos);

        Trajectory parkTraj1 = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(10, 20), Math.toRadians(90))
                .build();

        CommandScheduler.getInstance().scheduleCommand(
                new FindAprilTagCommand(vision)
                        .then(new RunCommand(() -> {
//                            currentTag = vision.getTags().get(0);
                            drive.followTrajectory(parkTraj1);
                        }))
        );
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


