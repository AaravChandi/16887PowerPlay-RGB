package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class ScoopSubsystem extends Subsystem {
    public final Servo claw;

    public enum State {
        IN,
        OUT
    }

    private State state;

    public ScoopSubsystem(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, Constants.Scoop.kClawName);
        this.state = State.OUT;
        claw.scaleRange(0,1);
    }

    public void setState(State state) {
        this.state = state;
    }
    public void changeState() {
        if (this.state == State.OUT) this.state = State.IN;
        else
            this.state = State.OUT;
    }
    public boolean isClawOpen(){
        if (this.state == State.OUT)
            return true;
        else
            return false;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("Position", claw.getPosition());
        switch (state) {
            case IN:
                claw.setPosition(Constants.Scoop.kIn);
                telemetry.addData("In position", claw.getPosition());
                break;
            case OUT:
                claw.setPosition(Constants.Scoop.kOut);
                telemetry.addData("Out position", claw.getPosition());
                claw.getPosition();
                break;
        }
    }
}
