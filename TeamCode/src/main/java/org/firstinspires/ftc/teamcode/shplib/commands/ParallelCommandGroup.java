package org.firstinspires.ftc.teamcode.shplib.commands;

import java.util.ArrayList;
import java.util.Arrays;

public class ParallelCommandGroup extends Command {
    private final ArrayList<Command> commands = new ArrayList<>();

    public ParallelCommandGroup(Command... commands) {
        this.commands.addAll(Arrays.asList(commands));
    }

    @Override
    public void init() {
        for (Command command : commands) {
            CommandScheduler.getInstance().scheduleCommand(command);
        }
    }
}
