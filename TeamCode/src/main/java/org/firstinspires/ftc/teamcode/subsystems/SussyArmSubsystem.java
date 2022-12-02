package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class SussyArmSubsystem extends Subsystem {

    public final SHPMotor linearSlide;

    public enum State {
        LOW,
        MEDIUM,
        HIGH,
    }

    private State state;
    private State prevState;

    public SussyArmSubsystem(HardwareMap hardwareMap){
        linearSlide = new SHPMotor(hardwareMap, "slide", MotorUnit.TICKS);
        linearSlide.reverseDirection();

        this.prevState = State.LOW;
        setState(State.LOW);
    }

    public void cycleState() {
        if (this.state == State.LOW) {
            this.prevState = State.LOW;
            setState(State.MEDIUM);
        } else if (this.state == State.MEDIUM && this.prevState == State.LOW) {
            this.prevState = State.MEDIUM;
            setState(State.HIGH);
        } else if (this.state == State.HIGH) {
            this.prevState = State.HIGH;
            setState(State.MEDIUM);
        } else if (this.state == State.MEDIUM && this.prevState == State.HIGH) {
            this.prevState = State.MEDIUM;
            setState(State.LOW);
        } else {
            this.prevState = State.LOW;
            setState(State.LOW);
        }
    }

    public void setState(State state) {

        this.state = state;
    }


    @Override
    public void periodic(Telemetry telemetry){
        switch (state) {
            case LOW:
                linearSlide.setPosition(50);
                break;
            case MEDIUM:
                linearSlide.setPosition(2900);
                break;
            case HIGH:
                linearSlide.setPosition(3825);
                break;
        }
    }

}
