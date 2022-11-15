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
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RRTankDrive;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.SHPMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@Autonomous

public class RightPosAuto extends BaseRobot {
    int currentTag;
    DriveSubsystem findOffset;
    RRMecanumDrive drive;
    Trajectory trajForward1, trajForward2, trajBack;

    @Override
    public void init() {
        //
        super.init();

        drive = new RRMecanumDrive(hardwareMap);
        findOffset = new DriveSubsystem(hardwareMap);
        //To get the current tag
        //currentTag.get(0);

        Pose2d startPos = new Pose2d(10, 10, Math.toRadians(90));
        drive.setPoseEstimate(startPos);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        trajForward1 = drive.trajectoryBuilder(startPos)
                .forward(60)
                .build();
        trajForward2 = drive.trajectoryBuilder(startPos)
                .forward(30)
                .build();
        trajBack = drive.trajectoryBuilder(startPos)
                .back(30)
                .build();

    }

    public void start() {
        super.start();

        CommandScheduler myCommand = CommandScheduler.getInstance();

        myCommand.scheduleCommand(
                new FindAprilTagCommand(vision)
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajForward1);
                                })
                        ).then(new WaitCommand(trajForward1.duration()))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajBack);
                                })
                        ).then(new WaitCommand(trajBack.duration()))
                        .then(new RunCommand(() ->{
                                    if(vision.getTags().get(0).id == 7) {
                                        drive.turn(Math.toRadians(-105));
                                        drive.followTrajectoryAsync(trajForward2);
                                    }
                                    else if(vision.getTags().get(0).id == 12) {
                                        drive.turn(Math.toRadians(115));
                                        drive.followTrajectoryAsync(trajForward2);
                                    }
                                })
                        )

                        .then(new RunCommand(() ->{
                                    findOffset.offset = drive.getCurrentAngle();
                        })));

    }



    @Override
    public void loop() {
        super.loop();
        for (AprilTagDetection tag : vision.getTags()) {
            telemetry.addData("Tag ID: ", tag.id);
        }

        PoseStorage.offset = drive.getCurrentAngle();

        drive.update();
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