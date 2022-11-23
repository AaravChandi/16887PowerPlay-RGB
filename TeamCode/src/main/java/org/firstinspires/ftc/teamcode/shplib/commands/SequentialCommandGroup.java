package org.firstinspires.ftc.teamcode.shplib.commands;

import java.util.ArrayList;
import java.util.Arrays;

public class SequentialCommandGroup extends Command {
    private final Command command;

    public SequentialCommandGroup(Command... commands) {
        ArrayList<Command> nextCommands = new ArrayList<>(Arrays.asList(commands).subList(1, commands.length));
        commands[0].then(nextCommands);
        this.command = commands[0];
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().scheduleCommand(command);
    }
}