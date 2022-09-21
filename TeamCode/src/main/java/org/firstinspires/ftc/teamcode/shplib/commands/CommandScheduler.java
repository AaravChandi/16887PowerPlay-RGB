package org.firstinspires.ftc.teamcode.shplib.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public final class CommandScheduler {
    private static CommandScheduler instance;

    private final ArrayList<Command> commands = new ArrayList<>();
    private final ArrayList<Subsystem> subsystems = new ArrayList<>();

    private Telemetry telemetry;

    public static CommandScheduler getInstance() {
        if (instance == null) instance = new CommandScheduler();
        return instance;
    }

    public static void resetInstance() {
        instance = null;
    }

    public void setTelemetry(Telemetry telemetry) {
        telemetry.clearAll();
        this.telemetry = telemetry;
    }

    public void registerSubsystem(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    public void scheduleCommand(Command command) {
        for (Command c : commands) {
            if (c.getClass().equals(command.getClass())
                    && !c.getClass().equals(RunCommand.class)
                    && !c.getClass().equals(ParallelCommandGroup.class))
                return;
        }
        command.init();
        commands.add(command);
        for (Command c : command.getWithCommands()) {
            scheduleCommand(c);
        }
    }

    // probably not the most efficient way of scheduling, but works pretty well for what we need
    public void run() throws InterruptedException {
        ArrayList<Subsystem> idleSubsystems = new ArrayList<>(subsystems);

        for (int i = 0; i < commands.size(); i++) {
            Command command = commands.get(i);

            // check if command subsystems are idle
            Subsystem[] commandSubsystems = command.getSubsystems();
            boolean canRun = true;
            for (Subsystem subsystem : commandSubsystems) {
                // if subsystem is not idle, it cannot execute
                if (!idleSubsystems.contains(subsystem)) {
                    canRun = false;
                    break;
                }
            }
            if (!canRun) continue;

            command.execute();

            for (Subsystem subsystem : commandSubsystems) {
                idleSubsystems.remove(subsystem);
            }

            if (command.isFinished()) {
                command.end();
                ArrayList<Command> nextCommands = command.getNextCommands();
                commands.remove(i);
                i--;

                if (nextCommands.size() > 0) {
                    Command nextCommand = nextCommands.remove(0);
                    nextCommand.then(nextCommands);
                    scheduleCommand(nextCommand);
                }
            }
        }

        // for any idle subsystems, run their default command
        for (Subsystem subsystem : idleSubsystems) {
            Command defaultCommand = subsystem.getDefaultCommand();
            if (defaultCommand != null) defaultCommand.execute();
        }

        for (Subsystem subsystem : subsystems) {
            subsystem.periodic(telemetry);
        }


//        for (int i = 0; i < commands.size(); i++) {
//            Command command = commands.get(i);
//            command.execute();
//            if (command.isFinished()) {
//                command.end();
//                commands.remove(i);
//                i--;
//            }
//        }

        // collapse subsystem current commands into array and then execute array, delete above
        // essentially gather commands to execute on each loop, including default commands

//        ArrayList<Command> commandsToRun = new ArrayList<>();
//
//        for (Subsystem subsystem : subsystems) {
//            subsystem.periodic(telemetry);
//
//            Command command = subsystem.getCurrentCommand();
//            // if current command is null, switch to default
//            if (command == null) {
//                command = subsystem.resetToDefault();
//                // if default is also null, continue to next subsystem (no commands to run for subsystem)
//                if (command == null) continue;
//            }
//
//            // if the command is finished, execute it if it hasnt, and reset to default command and continue
//            // doing this to avoid circular dependencies down the road
//            if (command.isFinished() && !command.hasShutdown()) {
//                command.execute();
//                command.end();
//                command.shutdown();
//                subsystem.resetToDefault();
//                continue;
//            }
//
//            // if command hasnt been added yet, add it
//            if (!commandsToRun.contains(command)) {
//                commandsToRun.add(command);
//            }
//        }
//
//        for (Command command : commandsToRun) {
//            command.execute();
//        }
    }
}
