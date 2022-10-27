package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
//import org.firstinspires.ftc.teamcode.commands.DumpCargoCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

@TeleOp
public class TestTeleOp extends BaseRobot {
    public SHPMotor slide;

    @Override
    public void init() {
        super.init();
        slide = new SHPMotor(hardwareMap, Constants.Arm.kSlideName, MotorUnit.TICKS);
        slide.enablePositionPID(Constants.Arm.kSlideP);
        slide.setPositionErrorTolerance(Constants.Arm.kSlideTolerance);

        // Default command runs when no other commands are scheduled for the subsystem
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );
    }

    @Override
    public void start() {
        super.start();
        slide.resetEncoder();


        // Add anything that needs to be run a single time when the OpMode starts
    }

    @Override
    public void loop() {

        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();
        if (gamepad1.a) slide.setPosition(Constants.Arm.kSlideMiddle);
        telemetry.addData("slide: ", slide.getPosition(MotorUnit.TICKS));

//        new Trigger(gamepad1.dpad_up, new MoveArmCommand(arm, MoveArmCommand.Direction.UP));
//        new Trigger(gamepad1.dpad_down, new MoveArmCommand(arm, MoveArmCommand.Direction.DOWN));
//        new Trigger(gamepad1.b && !arm.atBottom(), new DumpCargoCommand(scoop));
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

        new Trigger(gamepad1.x,
                new RunCommand((() -> {
                    arm.setState(ArmSubsystem.State.TOP);}), arm)
                        .then(new DriveCommand(drive, 0.1, 0.1, 0.1, 1))
                //        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))
                //        .then(new DumpCargoCommand(scoop))
                //.then(new MoveArmCommand(arm, MoveArmCommand.Direction.MIDDLE))
                //.then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
        );

        new Trigger(gamepad1.y,
                new RunCommand((() -> {
                    arm.setState(ArmSubsystem.State.BOTTOM);}), arm)
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))
                        .then(new DriveCommand(drive, 0, 0.1, 0, 1))
//                        .then(new DumpCargoCommand(scoop))
                //.then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
        );

        new Trigger(gamepad1.right_bumper,
                new RunCommand((() -> {drive.mecanum(1,1,1);}), drive)
                        .then(new DriveCommand(drive,0,0.1,0,  0.5))
                        .then(new RunCommand(() -> {
                            slide.setPosition(100);
                        }))
                        .then(new DriveCommand(drive,0,-0.1,0,  0.5))
                        .then(new RunCommand(() -> {
                            slide.setPosition(10);
                        }))

                //.then(new DriveCommand(drive,0,0,0.5,2))
                //.then(new DriveCommand(drive))

        );

        new Trigger(gamepad1.left_bumper,
                new RunCommand((() -> {drive.normalmecanum(1,1,1);}), drive)
                        .then(new DriveCommand(drive,0.5,0,0,0.9))
                        .then(new DriveCommand(drive,0,0,0,2))
                        .then(new DriveCommand(drive,0,0.5,0,1))
        );


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
