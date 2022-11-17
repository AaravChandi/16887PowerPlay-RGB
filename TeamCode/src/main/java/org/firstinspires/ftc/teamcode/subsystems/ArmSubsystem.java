package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.controllers.ElevatorFFController;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

// Use this class as a reference for creating new subsystems

public class ArmSubsystem extends Subsystem {
    public final SHPMotor slide;
    public boolean override;
    public int coneLevel;
    private double manualPos;

    public enum State {
        TOP,
        TOP_OF_TOP,
        MIDDLE,
        BOTTOM,
        SHORT,
        TOP_OF_SHORT,
        TOP_OF_MIDDLE,
        CARRYING,
        STACKED_CONES,
        MANUAL,
    }

    private State state;

    private double previousTime;

    public ArmSubsystem(HardwareMap hardwareMap) {
        slide = new SHPMotor(hardwareMap, Constants.Arm.K_SLIDE_NAME, MotorUnit.TICKS);
        slide.reverseDirection();
        slide.enablePositionPID(Constants.Arm.K_SLIDE_P);
        slide.setPositionErrorTolerance(Constants.Arm.K_SLIDE_TOLERANCE);
        slide.enableFF(new ElevatorFFController(0, Constants.Arm.K_SLIDE_G));
        coneLevel = 5;
//        slide.enableVelocityPID(Constants.Arm.kSlideP);
        //slide.enableProfiling(Constants.Arm.kSlideMaxVelocity);


        manualPos = 0;
        previousTime = Clock.now();
        setState(State.BOTTOM);
    }

    public void resetEncoder() {
        slide.resetEncoder();
    }

    public void setState(State state) {
        if (state == State.STACKED_CONES)
            incrementConeLevelDown();
        this.state = state;
        previousTime = Clock.now();
    }

    public void nextState() {
        if (this.state == State.MIDDLE) setState(State.TOP);
        else if (this.state == State.SHORT) setState(State.MIDDLE);
        else if (this.state == State.CARRYING) setState(State.SHORT);
        else if (this.state == State.BOTTOM) setState(State.CARRYING);
    }

    public State getState() {
        return state;
    }

    public void incrementConeLevelDown() {
        coneLevel--;
    }
    public void incrementConeLevelUp() {
        coneLevel++;
    }

    public void previousState() {
        if (this.state == State.TOP) setState(State.TOP_OF_TOP);
        else if (this.state == State.MIDDLE) setState(State.TOP_OF_MIDDLE);
        else if (this.state == State.SHORT) setState(State.TOP_OF_SHORT);
        else if (this.state == State.CARRYING) setState(State.BOTTOM);

    }

    public boolean atBottom() {
        return this.state == State.BOTTOM;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("slide ticks: ", slide.getPosition(MotorUnit.TICKS));
        telemetry.addData("time: ", Clock.elapsed(previousTime));
//        telemetry.addData("profile output: ", slide.followProfile(Clock.elapsed(previousTime)));
        if (!override) {
            switch (state) {
                case TOP:
                    slide.setPosition(Constants.Arm.K_SLIDE_TOP);
                    telemetry.addData("state: ", "TOP");
                    break;
                case CARRYING:
                    slide.setPosition(Constants.Arm.K_SLIDE_CARRY);
                    telemetry.addData("state: ", "Carrying");
                    break;
                case MIDDLE:
                    slide.setPosition(Constants.Arm.K_SLIDE_MIDDLE);
                    telemetry.addData("state: ", "Middle");
                    break;
                case TOP_OF_MIDDLE:
                    slide.setPosition(Constants.Arm.K_SLIDE_MIDDLE - 600);
                    break;
                case BOTTOM:
                    slide.setPosition(Constants.Arm.K_SLIDE_BOTTOM);
                    telemetry.addData("state: ", "BOTTOM");
                    break;
                case SHORT:
                    slide.setPosition(Constants.Arm.K_SLIDE_SHORT);
                    break;
                case TOP_OF_SHORT:
                    slide.setPosition(Constants.Arm.K_SLIDE_SHORT - 400);
                    break;
                case TOP_OF_TOP:
                    slide.setPosition(Constants.Arm.K_SLIDE_TOP - 500);
                    break;
                case STACKED_CONES:
                    slide.setPosition(Constants.Arm.K_SLIDE_BOTTOM + coneLevel*200);
                    break;
                case MANUAL:
                    slide.setPosition(manualPos);
                    break;
            }
        }
        else {
            telemetry.addData("Current Power", slide.getPower());
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
