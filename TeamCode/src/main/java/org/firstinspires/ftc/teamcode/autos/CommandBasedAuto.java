package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DumpCargoCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;

@Autonomous
public class CommandBasedAuto extends BaseRobot {
    @Override
    public void start() {
        CommandScheduler.getInstance().scheduleCommand(
                new MoveArmCommand(arm, MoveArmCommand.Direction.TOP)
                        .then(new DumpCargoCommand(scoop))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
        );
    }
}
