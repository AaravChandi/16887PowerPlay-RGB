package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

public class DumpCargoCommand extends Command {
    private final ScoopSubsystem scoop;
    private double startTime;
    boolean pickedUp = false;

    public DumpCargoCommand(ScoopSubsystem scoop) {
        // Pass through any subsystems that are uninterruptible
        super(scoop);
        this.scoop = scoop;

    }

    @Override
    public void init() {
        if (Clock.hasElapsed(startTime, 1) && !pickedUp) scoop.setState(ScoopSubsystem.State.IN);
        else scoop.setState(ScoopSubsystem.State.OUT);
        pickedUp = !pickedUp;
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
