package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.InteractWithConeCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@Autonomous

public class BlindLeftSideAuto extends BaseRobot {
    int currentTag;
    DriveSubsystem findOffset;
    RRMecanumDrive drive;
    Trajectory trajStrafeRight1, trajForward1, trajShaftPoleApproach, trajStrafePoleRetreat,
            trajBack1, trajToStack, trajToBackToPole1, trajToBackToPole2, trajStrafeLeft;

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
                .strafeRight(-35)
                .build();
        trajForward1 = drive.trajectoryBuilder(startPos)
                .forward(52)
                .build();
        trajShaftPoleApproach = drive.trajectoryBuilder(startPos)
                .strafeLeft(-5)
                .build();
        trajStrafePoleRetreat = drive.trajectoryBuilder(startPos)
                .strafeRight(-5)
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

        claw.setState(ClawSubsystem.State.CLOSED);

    }

    public void start() {
        super.start();
//TODO: Do wait commands
        CommandScheduler.getInstance().scheduleCommand(
                new RunCommand(() -> {
                    claw.setState(ClawSubsystem.State.CLOSED);

                })
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
                        .then(new WaitCommand(trajForward1.duration()))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))
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
//                        .then(new RunCommand(() -> {
//                            drive.followTrajectoryAsync(trajStrafePoleRetreat);
//                        }))
//                        .then(new WaitCommand(trajStrafePoleRetreat.duration()))
//                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.SHORT))
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajBack1);
                        }))
                        // go to stack
                        .then(new WaitCommand(trajBack1.duration()))
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajToStack);
                        }))
                        .then(new WaitCommand(trajToStack.duration()))
                        .then (new RunCommand(() -> {
                            claw.setState(ClawSubsystem.State.CLOSED);
                        }))
                        .then (new RunCommand(() -> {
                            arm.setState(ArmSubsystem.State.TOP);
                        }))
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajToBackToPole1);
                        }))
                        .then(new WaitCommand(trajToBackToPole1.duration()))
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajToBackToPole2);
                        }))
                        .then(new WaitCommand(trajToBackToPole2.duration()))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajShaftPoleApproach);
                                })
                        ).then(new WaitCommand(trajShaftPoleApproach.duration()))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP_OF_TOP))
                        .then(new InteractWithConeCommand(claw, InteractWithConeCommand.State.OUT))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajStrafePoleRetreat);
                                })
                        ).then(new WaitCommand(trajStrafePoleRetreat.duration()))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))


                        // parking
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajBack1);
                        }))
                        .then(new WaitCommand(trajBack1.duration()))
                        .then(new RunCommand(() -> {
                                    drive.followTrajectoryAsync(trajStrafeLeft);
                                })
                        ) .then(new WaitCommand(trajStrafeLeft.duration()))
                        .then(new RunCommand(() -> {
                            drive.followTrajectoryAsync(trajBack1);
                        }))
                        .then(new WaitCommand(trajBack1.duration()))


        );



//                        .then(new RunCommand(() ->{
//                                    if(vision.getTags().get(0).id == 7) {
//                                        drive.followTrajectoryAsync(trajStrafeLeft);
//                                    }
//                                    else if(vision.getTags().get(0).id == 12) {
//                                        drive.followTrajectoryAsync(trajStrafeRight);
//                                    }
//                                })
//                        ));


    }



    @Override
    public void loop() {
        super.loop();
//        for (AprilTagDetection tag : vision.getTags()) {
//            telemetry.addData("Tag ID: ", tag.id);
//        }

        PoseStorage.offset = drive.getCurrentAngle();

        drive.update();
    }

}