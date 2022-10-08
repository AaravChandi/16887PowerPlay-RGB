package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveCommand extends Command {
    private final DriveSubsystem drive;
    private double startTime;

    public DriveCommand(DriveSubsystem drive) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(drive);
        this.drive = drive;
    }

    // Called once when the command is initially schedule
    @Override
    public void init() {
        startTime = Clock.now();
        drive.mecanum(0.5, 0, 0);

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
        return Clock.hasElapsed(startTime, 1);
    }
}
