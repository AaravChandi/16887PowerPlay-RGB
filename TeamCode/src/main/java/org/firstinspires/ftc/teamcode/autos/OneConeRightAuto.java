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
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.openftc.apriltag.AprilTagDetection;

/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@Autonomous

public class OneConeRightAuto extends BaseRobot {
    int currentTag;
    DriveSubsystem findOffset;
    RRMecanumDrive drive;
    Trajectory trajStrafeLeft1, trajForward1, trajShaftPoleApproach, trajStrafePoleRetreat,
            trajBack1, trajToStack, trajToBackToPole1, trajToBackToPole2, trajStrafeLeft,
            trajFrontPark, trajPark3, trajPark2, trajTagFor, trajTagBack;

    @Override
    public void init() {
        //
        super.init();

        arm.resetEncoder();
        drive = new RRMecanumDrive(hardwareMap);
        //To get the current tag
        //currentTag.get(0);

        Pose2d startPos = new Pose2d(10, 10, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        trajTagFor = drive.trajectoryBuilder(startPos)
                .forward(10)
                .build();
        trajTagBack = drive.trajectoryBuilder(startPos)
                .back(10)
                .build();
        trajStrafeLeft1 = drive.trajectoryBuilder(startPos)
                .strafeLeft(-33)
                .build();
        trajForward1 = drive.trajectoryBuilder(startPos)
                .forward(37)
                .build();
        trajShaftPoleApproach = drive.trajectoryBuilder(startPos)
                .strafeLeft(-3)
                .build();
        trajStrafePoleRetreat = drive.trajectoryBuilder(startPos)
                .strafeRight(-7)
                .build();
        trajBack1 = drive.trajectoryBuilder(startPos)
                .back(15)
                .build();
        trajToStack = drive.trajectoryBuilder(startPos)
                .strafeLeft(-50)
                .build();
        trajToBackToPole1 = drive.trajectoryBuilder(startPos)
                .strafeRight(-55)
                .build();
        trajToBackToPole2 = drive.trajectoryBuilder(startPos)
                .forward(20)
                .build();
        trajStrafeLeft = drive.trajectoryBuilder(startPos)
                .strafeLeft(-20)
                .build();
        trajFrontPark = drive.trajectoryBuilder(startPos)
                .forward(15)
                .build();
        trajPark2 = drive.trajectoryBuilder(startPos)
                .strafeRight(-30)
                .build();
        trajPark3 = drive.trajectoryBuilder(startPos)
                .strafeRight(-60)
                .build();


    }

    public void start() {
        super.start();
//TODO: Do wait commands????

        CommandScheduler.getInstance().scheduleCommand(
                new RunCommand(() -> {
                    claw.setState(ClawSubsystem.State.CLOSED);
                }).then(new WaitCommand(1))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.CARRYING))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajTagFor);
                                })
                        ).then(new WaitCommand(trajTagFor.duration()))
                        .then(new FindAprilTagCommand(vision))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajTagBack);
                                })
                        ).then(new WaitCommand(trajTagBack.duration()))
                        //position
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajStrafeLeft1);
                                })
                        )
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.CARRYING))

                        //cone
                        .then(new WaitCommand(trajStrafeLeft1.duration()))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajForward1);
                                })
                        )
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))
                        .then(new WaitCommand(trajForward1.duration()))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajShaftPoleApproach);
                                })
                        ).then(new WaitCommand(trajShaftPoleApproach.duration()))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP_OF_TOP))
                        .then(new WaitCommand(1))
                        .then(new InteractWithConeCommand(claw, InteractWithConeCommand.State.OUT))
                        .then(new WaitCommand(1))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajStrafePoleRetreat);
                                })
                        ).then(new WaitCommand(trajStrafePoleRetreat.duration()))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))

                        //park
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajFrontPark);
                        })).then(new WaitCommand(trajFrontPark.duration()))
                        .then(new RunCommand(() -> {
                            if(vision.getTags().get(0).id == 8){
                                drive.followTrajectoryAsync(trajPark2);
                            }else if (vision.getTags().get(0).id == 12){
                                drive.followTrajectoryAsync(trajPark3);
                            }
                        }))



        );

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

}