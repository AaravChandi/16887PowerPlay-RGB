package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
//import org.firstinspires.ftc.teamcode.commands.DumpCargoCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

@TeleOp
public class TestTeleOp extends BaseRobot {
private double debounce;
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
        telemetry.addData("slide ticks: ", arm.slide.getPosition(MotorUnit.TICKS));

    }

    @Override
    public void start() {
        super.start();
        debounce = Clock.now();
        arm.override = false;


        // Add anything that needs to be run a single time when the OpMode starts
    }

    @Override
    public void loop() {

        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();

        new Trigger(gamepad1.x,
                new RunCommand(( () -> {drive.imu.initialize();})));

        new Trigger(gamepad1.dpad_up, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP)));

        new Trigger(gamepad1.dpad_down, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM)));

        //KEEP FOR NOW INCASE AARAV CODE DOES NOT WORK WHICH IT PROBABLY WILL NOT
        /*new Trigger(gamepad1.right_bumper, new DumpCargoCommand(scoop, DumpCargoCommand.State.IN)
                .then(new WaitCommand(0.2))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.NEXT)));

        new Trigger(gamepad1.left_bumper, new MoveArmCommand(arm, MoveArmCommand.Direction.PREVIOUS)
                .then(new DumpCargoCommand(scoop, DumpCargoCommand.State.OUT))
        );*/

        new Trigger(gamepad1.a, new RunCommand(() -> {
            if (!Clock.hasElapsed(debounce, 0.3)) return;
            if (scoop.isClawOpen()) {
                if (arm.getState() == ArmSubsystem.State.BOTTOM) {
                    scoop.setState(ScoopSubsystem.State.IN);
                    CommandScheduler.getInstance().scheduleCommand(
                            new WaitCommand(0.3)
                            .then(new RunCommand(() -> {
                                arm.setState(ArmSubsystem.State.CARRYING);
                            })));
                }
                else
                    scoop.setState(ScoopSubsystem.State.IN);

            }
            else {
                arm.previousState();
                CommandScheduler.getInstance().scheduleCommand(
                        new WaitCommand(0.5)
                                .then(new RunCommand(() -> {
                                    scoop.setState(ScoopSubsystem.State.OUT);
                                })));
            }
            debounce = Clock.now();
        }));

        new Trigger(gamepad1.left_bumper, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.SHORT)));

        new Trigger(gamepad1.right_bumper, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.MIDDLE)));

        new Trigger(gamepad1.y, new MoveArmCommand(arm, MoveArmCommand.Direction.CARRYING));

        if (gamepad1.dpad_right) {
            arm.override = true;
            arm.slide.setPower(0.25);
        }
        else if (gamepad1.dpad_left) {
            arm.override = true;
            arm.slide.setPower(-0.25);
        }
        else if (arm.override)
            arm.slide.setPower(0);

        new Trigger(gamepad1.b, new RunCommand(() -> {
            if (!Clock.hasElapsed(debounce, 0.5)) return;
            arm.setState(ArmSubsystem.State.STACKED_CONES);
            debounce = Clock.now();
            if (Clock.hasElapsed(debounce, 0.5)) arm.incrementConeLevelDown();

        }));
        /*new Trigger(gamepad2.dpad_down, new RunCommand(( () -> {arm.incrementConeLevelDown(1);})));
        new Trigger(gamepad2.dpad_up, new RunCommand(( () -> {arm.incrementConeLevelDown(-1);})));*/

//
//        // Dump cargo macro

        /*new Trigger(gamepad1.a,
                new RunCommand((() -> {
                    arm.setState(ArmSubsystem.State.SHORT);}), arm)
                        // on the way up
                        .then (new WaitCommand(0.25))
                        .then(new DriveCommand(drive, 0, -0.2, 0, 0.5, true))
                        .then (new WaitCommand(0.25))
                        .then (new MoveArmCommand(arm, MoveArmCommand.Direction.TopOfShort))
                        .then (new WaitCommand(0.1))
                        .then(new DumpCargoCommand(scoop, DumpCargoCommand.State.OUT))

                        .then (new WaitCommand(0.1))
                        // on the way down
                        .then(new DriveCommand(drive, 0, 0.2, 0, 0.5, true))
                        .then (new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
        );*/
        /*
        new Trigger(gamepad1.b,
                new RunCommand((() -> {
                    arm.setState(ArmSubsystem.State.MIDDLE);}), arm)
                        // on the way up
                        .then (new WaitCommand(1))
                        .then(new DriveCommand(drive, 0, -0.2, 0, 0.5, true))
                        .then (new WaitCommand(1))
                        .then (new MoveArmCommand(arm, MoveArmCommand.Direction.TopOfMiddle))
                        .then (new WaitCommand(0.1))
                        .then(new DumpCargoCommand(scoop, DumpCargoCommand.State.OUT))

                        .then (new WaitCommand(0.1))
                        // on the way down
                        .then(new DriveCommand(drive, 0, 0.2, 0, 0.5, true))
                        .then (new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
        );*/

        //new Trigger(gamepad1.a, new NewDropCommand(scoop));



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
