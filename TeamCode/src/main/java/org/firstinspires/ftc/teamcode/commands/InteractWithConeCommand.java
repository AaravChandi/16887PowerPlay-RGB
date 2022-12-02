package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class InteractWithConeCommand extends Command {
    private final ClawSubsystem scoop;
    private State state;
    private double startTime;

    public enum State {
        IN,
        OUT
    }

    public InteractWithConeCommand(ClawSubsystem scoop, State state) {
        // Pass through any subsystems that are uninterruptible
        super(scoop);
        this.state = state;
        this.scoop = scoop;

    }

    @Override
    public void init() {
        if (state == State.IN) scoop.setState(ClawSubsystem.State.CLOSED);
        if (state == State.OUT) scoop.setState(ClawSubsystem.State.OPEN);

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
