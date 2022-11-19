package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DumpCargoCommand;
import org.firstinspires.ftc.teamcode.commands.FindAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.openftc.apriltag.AprilTagDetection;

/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@Autonomous

public class RightPosAuto extends BaseRobot {
    int currentTag;
    DriveSubsystem findOffset;
    RRMecanumDrive drive;
    Trajectory trajForward1, trajBack1, trajStrafeRight, trajStrafeLeft, trajShaftPoleApproach, trajStrafePoleRetreat;

    @Override
    public void init() {
        //
        super.init();

        drive = new RRMecanumDrive(hardwareMap);
        findOffset = new DriveSubsystem(hardwareMap);
        //To get the current tag
        //currentTag.get(0);

        Pose2d startPos = new Pose2d(10, 10, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        trajForward1 = drive.trajectoryBuilder(startPos)
                .forward(100)
                .build();
        trajBack1 = drive.trajectoryBuilder(startPos)
                .back(40)
                .build();
        trajStrafeRight = drive.trajectoryBuilder(startPos)
                .strafeLeft(-100)
                .build();
        trajStrafeLeft = drive.trajectoryBuilder(startPos)
                .strafeLeft(-100)
                .build();
        trajShaftPoleApproach = drive.trajectoryBuilder(startPos)
                .strafeLeft(-40)
                .build();
        trajStrafePoleRetreat = drive.trajectoryBuilder(startPos)
                .strafeRight(-40)
                .build();

    }

    public void start() {
        super.start();

        CommandScheduler myCommand = CommandScheduler.getInstance();

        myCommand.scheduleCommand(
                new FindAprilTagCommand(vision)
                        //position
                        //.then(new DumpCargoCommand(scoop, DumpCargoCommand.State.IN))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajForward1);
                                })
                        )

                        //cone
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))
                        .then(new WaitCommand(trajForward1.duration()))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajShaftPoleApproach);
                                })
                        ).then(new WaitCommand(trajShaftPoleApproach.duration()))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP_OF_TOP))
                        //.then(new DumpCargoCommand(scoop, DumpCargoCommand.State.OUT))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajStrafePoleRetreat);
                                })
                        ).then(new WaitCommand(trajStrafePoleRetreat.duration()))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))

                        //park
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajStrafePoleRetreat);
                                })
                        ).then(new WaitCommand(trajStrafePoleRetreat.duration()))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajBack1);
                                })
                        ).then(new WaitCommand(trajBack1.duration()))
                        .then(new RunCommand(() ->{
                                    if(vision.getTags().get(0).id == 7) {
                                        drive.followTrajectoryAsync(trajStrafeLeft);
                                    }
                                    else if(vision.getTags().get(0).id == 12) {
                                        drive.followTrajectoryAsync(trajStrafeRight);
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