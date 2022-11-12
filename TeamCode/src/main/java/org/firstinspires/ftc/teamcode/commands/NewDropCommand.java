package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

public class NewDropCommand extends Command {
    private final ScoopSubsystem scoop;
    private double startTime;


    public NewDropCommand(ScoopSubsystem scoop) {
        // Pass through any subsystems that are uninterruptible
        super(scoop);
        this.scoop = scoop;

    }

    @Override
    public void init() {
        /*if (state == State.IN) scoop.setState(ScoopSubsystem.State.IN);
        if (state == State.OUT) scoop.setState(ScoopSubsystem.State.OUT);*/

    }

    @Override
    public void execute() {
        scoop.changeState();
    }

    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, 0.5);
    }

}
