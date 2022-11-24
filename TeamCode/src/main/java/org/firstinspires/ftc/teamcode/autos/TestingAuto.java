package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.InteractWithConeCommand;
import org.firstinspires.ftc.teamcode.commands.FindAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.openftc.apriltag.AprilTagDetection;

/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@Autonomous

public class TestingAuto extends BaseRobot {
    int currentTag;
    DriveSubsystem findOffset;
    RRMecanumDrive drive;
    Trajectory trajForward1, trajForward2, trajBack1, trajStrafeRight, trajStrafeLeft, trajShaftPoleApproach, trajStrafePoleRetreat,
            trajToStack, trajToBackToPole;

    @Override
    public void init() {
        //
        super.init();

        arm.resetEncoder();
        drive = new RRMecanumDrive(hardwareMap);
        findOffset = new DriveSubsystem(hardwareMap);
        //To get the current tag
        //currentTag.get(0);

        Pose2d startPos = new Pose2d(10, 10, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        trajForward1 = drive.trajectoryBuilder(startPos)
                .forward(54)
                .build();
        trajForward2 = drive.trajectoryBuilder(startPos)
                .forward(15)
                .build();
        trajToStack = drive.trajectoryBuilder(startPos)
                .strafeRight(-30)
                .build();
        trajToBackToPole= drive.trajectoryBuilder(startPos)
                .strafeLeft(-30)
                .build();
        trajBack1 = drive.trajectoryBuilder(startPos)
                .back(20)
                .build();
        trajStrafeRight = drive.trajectoryBuilder(startPos)
                .strafeLeft(40)
                .build();
        trajStrafeLeft = drive.trajectoryBuilder(startPos)
                .strafeLeft(-40)
                .build();
        trajShaftPoleApproach = drive.trajectoryBuilder(startPos)
                .strafeLeft(-15)
                .build();
        trajStrafePoleRetreat = drive.trajectoryBuilder(startPos)
                .strafeRight(-15)
                .build();

        scoop.setState(ClawSubsystem.State.IN);

    }

    public void start() {
        super.start();

        CommandScheduler.getInstance().scheduleCommand(
                new RunCommand(() -> {
                    scoop.setState(ClawSubsystem.State.IN);

                })
                        .then(new WaitCommand(2))
                        .then(new FindAprilTagCommand(vision))
                        //position
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
                        .then(new InteractWithConeCommand(scoop, InteractWithConeCommand.State.OUT))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajStrafePoleRetreat);
                                })
                        ).then(new WaitCommand(trajStrafePoleRetreat.duration()))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))

                        //park
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajStrafePoleRetreat);
                                }))
                        .then(new WaitCommand(trajStrafePoleRetreat.duration()))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.SHORT))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajForward2);
                                }))
                        // go to stack
                        .then(new WaitCommand(trajForward2.duration()))
                        .then(new RunCommand(() -> {
                            drive.turn(Math.toRadians(180));
                        }))
                        .then(new WaitCommand(2.75))
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajToStack);
                        }))
                        .then(new WaitCommand(trajToStack.duration()))
                        .then (new RunCommand(() -> {
                            scoop.setState(ClawSubsystem.State.IN);
                        }))
                        .then (new RunCommand(() -> {
                            arm.setState(ArmSubsystem.State.TOP);
                        }))
                        .then(new RunCommand(() -> {
                            drive.turn(Math.toRadians(180));
                        }))
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajToBackToPole);
                        }))


                        // parking
                        .then(new RunCommand(() ->{
                                    if(vision.getTags().get(0).id == 7) {
                                        drive.followTrajectoryAsync(trajStrafeLeft);
                                    }
                                    else if(vision.getTags().get(0).id == 12) {
                                        drive.followTrajectoryAsync(trajStrafeRight);
                                    }
                                })
                        ));


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