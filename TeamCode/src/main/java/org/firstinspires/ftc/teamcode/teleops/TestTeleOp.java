package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.A;
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
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@TeleOp
public class TestTeleOp extends BaseRobot {
private double debounce;
private int desiredPosition;
private double maxSpeed;
private ArmSubsystem.State topState;
    @Override
    public void init() {
        super.init();

        // Default command runs when no other commands are scheduled for the subsystem
        drive.setDefaultCommand(
                new RunCommand(
                        () ->
                                drive.mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x)
                )
        );

        arm.resetEncoder();
        telemetry.addData("slide ticks: ", arm.slide.getPosition(MotorUnit.TICKS));
        ArmSubsystem.State topState = ArmSubsystem.State.TOP;

        new RunCommand(( () -> {arm.setState(ArmSubsystem.State.BOTTOM);}));

    }

    @Override
    public void start() {
        super.start();
        debounce = Clock.now();
        arm.override = false;
        maxSpeed = 0.45;
        topState = ArmSubsystem.State.TOP;


        // Add anything that needs to be run a single time when the OpMode starts
    }


    @Override
    public void loop() {

        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();
        //drive.setDriveBias(arm.getDriveBias(), maxSpeed);
        telemetry.addData("max speed: ", maxSpeed);
        new Trigger(gamepad1.y,
                new RunCommand(( () -> {drive.imu.initialize();})));

        new Trigger(gamepad1.dpad_up, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))
                .then(new RunCommand(( () -> {topState = ArmSubsystem.State.TOP;}))));

        new Trigger(gamepad1.dpad_down, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
                .then(new RunCommand(( () -> {topState = ArmSubsystem.State.BOTTOM;}))));


        //setting top position
        new Trigger(gamepad2.a,
                new RunCommand(( () -> {topState = ArmSubsystem.State.CARRYING;})));
        new Trigger(gamepad2.b,
                new RunCommand(( () -> {topState = ArmSubsystem.State.SHORT;})));
        new Trigger(gamepad2.x,
                new RunCommand(( () -> {topState = ArmSubsystem.State.MIDDLE;})));
        new Trigger(gamepad2.y,
                new RunCommand(( () -> {topState = ArmSubsystem.State.TOP;})));

        //KEEP FOR NOW INCASE AARAV CODE DOES NOT WORK WHICH IT PROBABLY WILL NOT
        /*new Trigger(gamepad1.right_bumper, new DumpCargoCommand(scoop, DumpCargoCommand.State.IN)
                .then(new WaitCommand(0.2))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.NEXT)));

        new Trigger(gamepad1.left_bumper, new MoveArmCommand(arm, MoveArmCommand.Direction.PREVIOUS)
                .then(new DumpCargoCommand(scoop, DumpCargoCommand.State.OUT))
        );*/

        new Trigger(gamepad1.b, new RunCommand(() -> {
            if (!Clock.hasElapsed(debounce, 0.3)) return;
            if (claw.isClawOpen()) {
                if (arm.getState() == ArmSubsystem.State.BOTTOM) {
                    claw.setState(ClawSubsystem.State.CLOSED);
                    CommandScheduler.getInstance().scheduleCommand(
                            new WaitCommand(0.3)
                            .then(new RunCommand(() -> {
                                arm.setState(ArmSubsystem.State.CARRYING);
                            })));
                }
                else if (arm.getState() == ArmSubsystem.State.STACKED_CONES)
                {
                    claw.setState(ClawSubsystem.State.CLOSED);
                    CommandScheduler.getInstance().scheduleCommand(
                            new WaitCommand(0.3)
                                    .then(new RunCommand(() -> {
                                        arm.setState(ArmSubsystem.State.SHORT);
                                    })));
                }
                else {
                    claw.setState(ClawSubsystem.State.CLOSED);
                    arm.nextState();
                }

            }
            else {
                claw.setState(ClawSubsystem.State.OPEN);
            }
            debounce = Clock.now();
        }));



        /*new Trigger(gamepad1.a, new RunCommand(() -> {
            if (!Clock.hasElapsed(debounce, 0.5)) return;
            if (claw.isClawOpen() && (arm.getState() == ArmSubsystem.State.BOTTOM)) {
                    claw.setState(ClawSubsystem.State.CLOSED);
                    CommandScheduler.getInstance().scheduleCommand(
                            new WaitCommand(0.3)
                            .then(new RunCommand(() -> {
                                arm.setState(ArmSubsystem.State.TOP);
                            })));
            }
            else if (!claw.isClawOpen() && arm.getState() == ArmSubsystem.State.BOTTOM) {
                claw.setState(ClawSubsystem.State.OPEN);
            }
            else if (!claw.isClawOpen() && (arm.getState() == ArmSubsystem.State.TOP || arm.getState() == ArmSubsystem.State.MANUAL)) {
                claw.setState(ClawSubsystem.State.OPEN);
            }
            else if (claw.isClawOpen() && (arm.getState() == ArmSubsystem.State.TOP || arm.getState() == ArmSubsystem.State.MANUAL)){
                arm.setState(ArmSubsystem.State.BOTTOM);
            }
            debounce = Clock.now();

        }));*/

        new Trigger(gamepad1.a, new RunCommand(() -> {
            if (!Clock.hasElapsed(debounce, 0.5)) return;
            if (arm.getState() == ArmSubsystem.State.STACKED_CONES) {
                claw.setState(ClawSubsystem.State.CLOSED);
                CommandScheduler.getInstance().scheduleCommand(
                        new WaitCommand(0.3)
                                .then(new RunCommand(() -> {
                                    arm.setState(ArmSubsystem.State.SHORT);

                                })));
            }
            else if (claw.isClawOpen() && (arm.getState() == ArmSubsystem.State.BOTTOM)) {
                claw.setState(ClawSubsystem.State.CLOSED);
                CommandScheduler.getInstance().scheduleCommand(
                        new WaitCommand(0.3)
                                .then(new RunCommand(() -> {
                                    arm.setState(topState);

                                })));
            }
            else if (!claw.isClawOpen() && arm.getState() == ArmSubsystem.State.BOTTOM) {
                claw.setState(ClawSubsystem.State.OPEN);
            }
            else if (!claw.isClawOpen() && arm.getState() != ArmSubsystem.State.BOTTOM) {    // (arm.getState() == topState || (!claw.isClawOpen() && arm.getState() == ArmSubsystem.State.MANUAL))
                claw.setState(ClawSubsystem.State.OPEN);
                CommandScheduler.getInstance().scheduleCommand(
                        new WaitCommand(0.5)
                                .then(new RunCommand(() -> {
                                    arm.setState(ArmSubsystem.State.BOTTOM);
                                })));

            }
            else if (claw.isClawOpen() && (arm.getState() == topState  || arm.getState() == ArmSubsystem.State.MANUAL)){
                arm.setState(ArmSubsystem.State.BOTTOM);
            }
            debounce = Clock.now();

        }));

        new Trigger(gamepad1.dpad_left, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.SHORT))
                .then(new RunCommand(( () -> {topState = ArmSubsystem.State.SHORT;}))));

        new Trigger(gamepad1.right_trigger>0.5, new RunCommand(( () -> {drive.setDriveBias(arm.getDriveBias(), 0.55);})));
        new Trigger(gamepad1.right_trigger <= 0.5, new RunCommand(( () -> {drive.setDriveBias(arm.getDriveBias(), 0);})));

        new Trigger(gamepad1.dpad_right, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.MIDDLE))
                .then(new RunCommand(( () -> {topState = ArmSubsystem.State.MIDDLE;}))));

        new Trigger(gamepad1.right_bumper, new RunCommand(( () -> {
            desiredPosition = (int)(arm.slide.getPosition(MotorUnit.TICKS)) + 200;
            arm.setManualPos(desiredPosition);
        })));
        new Trigger(gamepad1.left_bumper, new RunCommand(( () -> {
            desiredPosition = (int)(arm.slide.getPosition(MotorUnit.TICKS)) - 200;
            arm.setManualPos(desiredPosition);
        })));

        new Trigger(gamepad1.x, new RunCommand(() -> {
            if (!Clock.hasElapsed(debounce, 0.5)) return;
            arm.setState(ArmSubsystem.State.STACKED_CONES);
            debounce = Clock.now();
            if (Clock.hasElapsed(debounce, 0.5)) arm.incrementConeLevelDown();

        }));

    }

}
