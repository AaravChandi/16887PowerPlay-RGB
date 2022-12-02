package org.firstinspires.ftc.teamcode.teleops;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class UrMomDrive extends BaseRobot {
    public UrMomDrive() {
        super();
    }

    @Override
    public void init() {
        super.init();
        // Default command runs when no other commands are scheduled for the subsystem
        drive.setDefaultCommand(
                new RunCommand(() -> drive.mecanum(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x
                )));
        arm.resetEncoder();
    }

    @Override
    public void loop() {
        super.loop();

        new Trigger(gamepad1.dpad_up,
                new RunCommand(( () -> {susArm.cycleState();})));

        new Trigger(gamepad1.a,
                new RunCommand(( () -> {claw.setState(ClawSubsystem.State.IN);})));

        new Trigger(gamepad1.b,
                new RunCommand(( () -> {claw.setState(ClawSubsystem.State.OUT);})));
    }
}
