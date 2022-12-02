package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class ClawSubsystem extends Subsystem {
    public final Servo claw;

    public enum State {
        CLOSED,
        OPEN
    }

    private State state;

    public ClawSubsystem(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, Constants.Scoop.K_CLAW_NAME);
        this.state = State.OPEN;
        claw.scaleRange(0,1);
        //claw.setDirection(Servo.Direction.REVERSE);
    }

    public void setState(State state) {
        this.state = state;
    }
    public void changeState() {
        if (this.state == State.OPEN) this.state = State.CLOSED;
        else
            this.state = State.OPEN;
    }
    public boolean isClawOpen(){
        if (this.state == State.OPEN)
            return true;
        else
            return false;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("Position", claw.getPosition());
        switch (state) {
            case CLOSED:
                claw.setPosition(Constants.Scoop.K_IN);
                telemetry.addData("In position", claw.getPosition());
                break;
            case OPEN:
                claw.setPosition(Constants.Scoop.K_OUT);
                telemetry.addData("Out position", claw.getPosition());
                claw.getPosition();
                break;
        }
    }
}
