package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class ScoopSubsystem extends Subsystem {
    public final Servo scoop;

    public enum State {
        TOP,
        MIDDLE,
        BOTTOM
    }

    private State state;

    public ScoopSubsystem(HardwareMap hardwareMap) {
        scoop = hardwareMap.get(Servo.class, Constants.Scoop.kScoopName);

        this.state = State.BOTTOM;
    }

    public void setState(State state) {
        this.state = state;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        switch (state) {
            case TOP:
                scoop.setPosition(Constants.Scoop.kTop);
                break;
            case MIDDLE:
                scoop.setPosition(Constants.Scoop.kMiddle);
                break;
            case BOTTOM:
                scoop.setPosition(Constants.Scoop.kBottom);
                break;
        }
    }
}
