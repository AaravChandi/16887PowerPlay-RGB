package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.InteractWithConeCommand;
import org.firstinspires.ftc.teamcode.commands.FindAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.openftc.apriltag.AprilTagDetection;

/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@Autonomous

public class LeftPosAuto extends BaseRobot {
    int currentTag;

    Trajectory trajForward1, trajForward2, trajBack;

    @Override
    public void init() {
        //
        super.init();
        scoop.setState(ClawSubsystem.State.IN);


        //To get the current tag
        //currentTag.get(0);

    }

    public void start() {
        super.start();

        CommandScheduler myCommand = CommandScheduler.getInstance();

        myCommand.scheduleCommand(
                new FindAprilTagCommand(vision)
                        .then(new DriveCommand(drive,0.5, 0.0, 0, 1.75, false))
                .then (new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))
                .then (new WaitCommand(2))
                .then(new DriveCommand(drive,0, -0.275, 0, 0.3, false))
                .then (new WaitCommand(2))
                .then (new MoveArmCommand(arm, MoveArmCommand.Direction.TOP_OF_TOP))
                .then (new InteractWithConeCommand(scoop, InteractWithConeCommand.State.OUT))
                .then (new WaitCommand(2))
                .then(new DriveCommand(drive,0, 0.275, 0, 0.4, false))
                        .then (new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
                .then (new WaitCommand(2))
                .then(new DriveCommand(drive,-0.5, 0.0, 0, 2.2, false))
                .then(new RunCommand(() ->{
                            if(vision.getTags().get(0).id == 7) {
                                myCommand.scheduleCommand(new DriveCommand(drive,0, -0.5, 0, 2.75, false)
                                        .then (new DriveCommand(drive,0, 0, 0, 2.2, false)));
                            }
                            else if(vision.getTags().get(0).id == 12) {
                                myCommand.scheduleCommand(new DriveCommand(drive,0, 0.5, 0, 2.75, false)
                                        .then (new DriveCommand(drive,0, 0, 0, 2.2, false)));
                            }
                        })
                )
        );

    }

    @Override
    public void loop() {
        super.loop();
        for (AprilTagDetection tag : vision.getTags()) {
            telemetry.addData("Tag ID: ", tag.id);
        }




    }

}