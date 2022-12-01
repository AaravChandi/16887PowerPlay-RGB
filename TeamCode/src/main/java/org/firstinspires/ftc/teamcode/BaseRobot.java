package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * Template created by Ayaan Govil on 8/21/2021.
 * <p>
 * FTC Java Documentation: http://ftctechnh.github.io/ftc_app/doc/javadoc/index.html
 * <p>
 * Helpful Shortcuts:
 * - Ctrl/Command + / = Comment/Uncomment line (can highlight multiple lines)
 * - Ctrl/Command + B = Go to declaration
 * - Ctrl/Command + Alt/Option + L = Auto format code
 */

// Previous gradle: 4.0.1

public class BaseRobot extends OpMode {
    // Declare subsystems and devices
    public DriveSubsystem drive;
    public VisionSubsystem vision;
    public ArmSubsystem arm;
    public ClawSubsystem claw;

    public SHPMotor intake;

    public double previousTime = 0;

    // Called when you press the init button
    @Override
    public void init() {
        // Starts universal clock - DO NOT DELETE!
        Clock.start();

        // Assigns telemetry object for Subsystem.periodic - DO NOT DELETE!
        CommandScheduler.getInstance().setTelemetry(telemetry);

        // Initialize your subsystems and devices
        drive = new DriveSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);
//        intake = new SHPMotor(hardwareMap, "intake");
//        scissorsystem = new ScissorSubsystem(hardwareMap);

        //claw = hardwareMap.get(Servo.class, "claw");
    }

    // Called when you press the start button
    @Override
    public void start() {

    }

    // Called repeatedly while an OpMode is running
    @Override
    public void loop() {
        telemetry.addData("Loop Time (ms): ", Clock.elapsed(previousTime) * 1000);
        telemetry.addData("Number of Stacked Cones Left", arm.coneLevel);
        previousTime = Clock.now();

        // Handles all subsystem and command execution - DO NOT DELETE!
        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    // Called when you press the stop button
    @Override
    public void stop() {
        // Flushes any cached subsystems and commands - DO NOT DELETE!
        CommandScheduler.resetInstance();
    }
}
