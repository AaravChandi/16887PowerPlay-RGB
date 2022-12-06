package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
//import org.firstinspires.ftc.teamcode.commands.DumpCargoCommand;
import org.firstinspires.ftc.teamcode.TestBaseRobot;
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
public class ControlHubTest extends TestBaseRobot {
    @Override
    public void init() {
        super.init();

        // Default command runs when no other commands are scheduled for the subsystem


        arm.resetEncoder();
        telemetry.addData("slide ticks: ", arm.slide.getPosition(MotorUnit.TICKS));

    }

    @Override
    public void start() {
        super.start();

        arm.override = false;


        // Add anything that needs to be run a single time when the OpMode starts
    }

    @Override
    public void loop() {

        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();

        new Trigger(gamepad1.dpad_up, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP)));

        new Trigger(gamepad1.dpad_down, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM)));


    }

}
