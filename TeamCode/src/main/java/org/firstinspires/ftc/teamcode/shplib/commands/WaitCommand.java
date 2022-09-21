package org.firstinspires.ftc.teamcode.shplib.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitCommand extends Command {
    private final double seconds;
    private ElapsedTime timer;

    public WaitCommand(double seconds) {
        this.seconds = seconds;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= seconds;
    }
}
