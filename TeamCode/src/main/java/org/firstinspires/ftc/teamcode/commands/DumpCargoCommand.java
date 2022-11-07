package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

public class DumpCargoCommand extends Command {
    private final ScoopSubsystem scoop;
    private State state;
    private double startTime;

    public enum State {
        IN,
        OUT
    }

    public DumpCargoCommand(ScoopSubsystem scoop, State state) {
        // Pass through any subsystems that are uninterruptible
        super(scoop);
        this.state = state;
        this.scoop = scoop;

    }

    @Override
    public void init() {
        if (state == State.IN) scoop.setState(ScoopSubsystem.State.IN);
        if (state == State.OUT) scoop.setState(ScoopSubsystem.State.OUT);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
