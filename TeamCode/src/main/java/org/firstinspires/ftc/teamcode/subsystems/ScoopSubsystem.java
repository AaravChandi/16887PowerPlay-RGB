package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class ScoopSubsystem extends Subsystem {
    public final Servo scoop;

    public enum State {
        IN,
        OUT
    }

    private State state;

    public ScoopSubsystem(HardwareMap hardwareMap) {
        scoop = hardwareMap.get(Servo.class, Constants.Scoop.kScoopName);
        this.state = State.OUT;
    }

    public void setState(State state) {
        this.state = state;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        switch (state) {
            case IN:
                scoop.setPosition(Constants.Scoop.kIn);
                break;
            case OUT:
                scoop.setPosition(Constants.Scoop.kOut);
                break;
        }
    }
}
