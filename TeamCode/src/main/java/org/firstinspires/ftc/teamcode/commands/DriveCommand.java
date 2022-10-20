package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveCommand extends Command {
    private final DriveSubsystem drive;
    private double startTime;
    private double endTime;
    double leftY; double leftX; double rightX; double time;

    public DriveCommand(DriveSubsystem drive, double leftY, double leftX, double rightX, double time) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(drive);
        this.drive = drive;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
        this.time = time;
    }

    // Called once when the command is initially schedule

    public void init() {
        startTime = Clock.now();
        drive.normalmecanum(leftY, leftX, rightX);
        endTime = time;

    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {

    }

    // Called once after isFinished() returns true
    @Override
    public void end() {

    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, time);
    }
}
