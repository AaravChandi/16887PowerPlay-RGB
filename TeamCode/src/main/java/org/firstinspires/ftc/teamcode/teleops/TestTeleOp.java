package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
//import org.firstinspires.ftc.teamcode.commands.DumpCargoCommand;
import org.firstinspires.ftc.teamcode.commands.DumpCargoCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

@TeleOp
public class TestTeleOp extends BaseRobot {

    @Override
    public void init() {
        super.init();

        // Default command runs when no other commands are scheduled for the subsystem
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );

        arm.resetEncoder();

    }

    @Override
    public void start() {
        super.start();


        // Add anything that needs to be run a single time when the OpMode starts
    }

    @Override
    public void loop() {

        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();

        //new Trigger(gamepad1.dpad_up, new MoveArmCommand(arm, MoveArmCommand.Direction.TOP));
        //new Trigger(gamepad1.dpad_down, new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM));
        new Trigger(gamepad1.dpad_left, new MoveArmCommand(arm, MoveArmCommand.Direction.MIDDLE));

        new Trigger(gamepad1.left_bumper, new DumpCargoCommand(scoop, DumpCargoCommand.State.OUT)
                .then (new WaitCommand(0.5))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM)));

        new Trigger(gamepad1.right_bumper, new DumpCargoCommand(scoop, DumpCargoCommand.State.IN)
                .then (new WaitCommand(0.5))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.MIDDLE)));
        new Trigger(gamepad1.dpad_up, new MoveArmCommand(arm, MoveArmCommand.Direction.TOP));
        new Trigger(gamepad1.dpad_down, new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM));
        new Trigger(gamepad1.dpad_right, new DumpCargoCommand(scoop, DumpCargoCommand.State.OUT));
        new Trigger(gamepad1.dpad_left, new DumpCargoCommand(scoop, DumpCargoCommand.State.IN));

//
//        // Dump cargo macro
        new Trigger(gamepad1.b,
                new RunCommand((() -> {
                    arm.setState(ArmSubsystem.State.TOP);}), arm)
                //.then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))
                //.then(new DumpCargoCommand(scoop))
                //.then(new MoveArmCommand(arm, MoveArmCommand.Direction.MIDDLE))
                //.then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
        );

        new Trigger(gamepad1.a,
                new RunCommand((() -> {
                    arm.setState(ArmSubsystem.State.TOP);}), arm)
                        // on the way up
                        .then (new WaitCommand(0.25))
                        .then(new DriveCommand(drive, 0, -0.4, 0, 0.75, true))
                        .then (new WaitCommand(0.25))
                        .then (new MoveArmCommand(arm, MoveArmCommand.Direction.TopOfShort))
                        .then (new WaitCommand(0.1))
                        .then(new DumpCargoCommand(scoop, DumpCargoCommand.State.OUT))

                        .then (new WaitCommand(0.1))
                        // on the way down
                        .then(new DriveCommand(drive, 0, 0.4, 0, 0.75, true))
                        .then (new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
        );


        /*new Trigger(gamepad1.left_bumper,
                new RunCommand((() -> {drive.normalmecanum(1,1,1);}), drive)
                        .then(new DriveCommand(drive,0.5,0,0,0.9))
                        .then(new DriveCommand(drive,0,0,0,2))
                        .then(new DriveCommand(drive,0,0.5,0,1))
        );*/


//
//        intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
//        if (gamepad1.a) {
//            claw.setPower(0.5);
//        } else if (gamepad1.b){
//            claw.setPower(-0.5);
//        } else claw.setPower(0.0);
//    }
    }

}
