package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class MoveArmCommand extends Command {
    private final ArmSubsystem arm;
    private final Direction direction;
    private double startTime;
    private double endTime;


    public enum Direction {
        TOP,
        TopOfTop,
        MIDDLE,
        BOTTOM,
        SHORT,
        TopOfShort,
        TopOfMiddle,
        CARRYING,
        NEXT,
        PREVIOUS,
    }

    public MoveArmCommand(ArmSubsystem arm, Direction direction) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(arm);
        this.arm = arm;
        this.direction = direction;
        if (direction == Direction.MIDDLE)
            endTime = 1;
        else if (direction == Direction.PREVIOUS)
            endTime = 0.5;
        else if (direction == Direction.TOP)
            endTime = 3;
        if (arm.slide.getPosition(MotorUnit.TICKS)<1000 && direction == Direction.BOTTOM)
            endTime = 1;
        else
            endTime = 1;
    }

    @Override
    public void init() {
        startTime = Clock.now();
        if (direction == Direction.TOP) {arm.setState(ArmSubsystem.State.TOP);}
        else if (direction == Direction.MIDDLE) arm.setState(ArmSubsystem.State.MIDDLE);
        else if (direction == Direction.BOTTOM) arm.setState(ArmSubsystem.State.BOTTOM);
        else if (direction == Direction.TopOfShort) arm.setState(ArmSubsystem.State.TopOfShort);
        else if (direction == Direction.TopOfMiddle) arm.setState(ArmSubsystem.State.TopOfMiddle);
        else if (direction == Direction.SHORT) arm.setState(ArmSubsystem.State.SHORT);
        else if (direction == Direction.CARRYING) arm.setState(ArmSubsystem.State.CARRYING);
        else if (direction == Direction.NEXT) arm.nextState();
        else if (direction == Direction.PREVIOUS) arm.previousState();
        else if (direction == Direction.TopOfTop) arm.setState(ArmSubsystem.State.TopOfTop);
    }

    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, endTime);
    }
}
