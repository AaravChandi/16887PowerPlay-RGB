package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
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

public class TwoConeLeftAuto extends BaseRobot {
    int currentTag;
    DriveSubsystem findOffset;
    RRMecanumDrive drive;
    Trajectory trajStrafeRight1, trajForward1, trajShaftPoleApproach, trajStrafePoleRetreat,
            trajBack1, trajToStack, trajToBackToPole1, trajToBackToPole2, trajStrafeLeft,
            trajBack, trajPark2, trajPark1, trajStrafeStack, trajForwardStack, trajStrafePole;
    //ignore the shitty names, readability is not a priority

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

        trajStrafeRight1 = drive.trajectoryBuilder(startPos)
                .strafeRight(-28)
                .build();
        trajForward1 = drive.trajectoryBuilder(startPos)
                .forward(48.5)
                .build();
        trajShaftPoleApproach = drive.trajectoryBuilder(startPos)
                .strafeLeft(-1)
                .build();
        trajStrafePoleRetreat = drive.trajectoryBuilder(startPos)
                .strafeRight(-1)
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
        trajBack = drive.trajectoryBuilder(startPos)
                .back(10)
                .build();
        trajPark2 = drive.trajectoryBuilder(startPos)
                .back(-25)
                .build();
        trajPark1 = drive.trajectoryBuilder(startPos)
                .back(-50)
                .build();
        trajStrafeStack = drive.trajectoryBuilder(startPos)
                .strafeLeft(-50)
                .build();
        trajForwardStack = drive.trajectoryBuilder(startPos)
                .forward(10)
                .build();
        trajStrafePole = drive.trajectoryBuilder(startPos)
                .strafeRight(-50)
                .build();

        claw.setState(ClawSubsystem.State.CLOSED);

    }

    public void start() {
        super.start();
//TODO: Do wait commands????

        CommandScheduler.getInstance().scheduleCommand(
                new FindAprilTagCommand(vision)
                        .then(new RunCommand(() -> {
                            claw.setState(ClawSubsystem.State.CLOSED);
                        }))
                        .then(new WaitCommand(3))
                        //position
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajStrafeRight1);
                                })
                        )
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.CARRYING))

                        //cone
                        .then(new WaitCommand(trajStrafeRight1.duration()))
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

                        //pick up cone
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajBack);
                        })).then(new WaitCommand(trajBack.duration()))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajStrafeStack);
                        })).then(new WaitCommand(trajStrafeStack.duration()))
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajForwardStack);
                        })).then(new WaitCommand(trajForwardStack.duration()))
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajShaftPoleApproach);
                        })).then(new WaitCommand(trajShaftPoleApproach.duration()))

                        .then(new InteractWithConeCommand(claw, InteractWithConeCommand.State.IN))
                        .then(new WaitCommand(2))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))

                        //back to pole
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajStrafePoleRetreat);
                        })).then(new WaitCommand(trajStrafePoleRetreat.duration()))
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajStrafePole);
                        })).then(new WaitCommand(trajStrafePole.duration()))
                        .then(new RunCommand(() -> {
                            drive.turn(90);
                        })).then(new WaitCommand(2))
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajShaftPoleApproach);
                        })).then(new WaitCommand(trajShaftPoleApproach.duration()))

                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP_OF_TOP))
                        .then(new WaitCommand(1))
                        .then(new InteractWithConeCommand(claw, InteractWithConeCommand.State.OUT))
                        .then(new WaitCommand(1))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajStrafePoleRetreat);
                                })
                        ).then(new WaitCommand(trajStrafePoleRetreat.duration()))

                        //park
                        .then(new RunCommand(() -> {
                            if(vision.getTags().get(0).id == 7){
                                drive.followTrajectoryAsync(trajPark1);
                            }else if (vision.getTags().get(0).id == 8){
                                drive.followTrajectoryAsync(trajPark2);
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