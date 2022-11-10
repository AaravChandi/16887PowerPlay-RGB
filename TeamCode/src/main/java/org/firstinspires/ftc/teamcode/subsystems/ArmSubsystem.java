package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
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

    public enum State {
        TOP,
        MIDDLE,
        BOTTOM,
        SHORT,
        TopOfShort,
        TopOfMiddle,
        CARRYING
    }

    private State state;

    private double previousTime;

    public ArmSubsystem(HardwareMap hardwareMap) {
        slide = new SHPMotor(hardwareMap, Constants.Arm.kSlideName, MotorUnit.TICKS);
        slide.reverseDirection();
        slide.enablePositionPID(Constants.Arm.kSlideP);
        slide.setPositionErrorTolerance(Constants.Arm.kSlideTolerance);
        slide.enableFF(new ElevatorFFController(0, Constants.Arm.kSlideG));
//        slide.enableVelocityPID(Constants.Arm.kSlideP);
        //slide.enableProfiling(Constants.Arm.kSlideMaxVelocity);



        previousTime = Clock.now();
        setState(State.BOTTOM);
    }

    public void resetEncoder() {
        slide.resetEncoder();
    }

    public void setState(State state) {
        this.state = state;
        previousTime = Clock.now();
//        if (state == State.TOP) {
//            slide.profileTo(Constants.Arm.kSlideTop);
//        } else if (state == State.MIDDLE) {
//            slide.profileTo(Constants.Arm.kSlideMiddle);
//        } else if (state == State.BOTTOM) {
//            slide.profileTo(Constants.Arm.kSlideBottom);
//        }
    }

    public void nextState() {
        if (this.state == State.MIDDLE) setState(State.TOP);
        else if (this.state == State.SHORT) setState(State.MIDDLE);
        else if (this.state == State.BOTTOM) setState(State.SHORT);
    }

    public void previousState() {
        if (this.state == State.TOP) setState(State.MIDDLE);
        else if (this.state == State.MIDDLE) setState(State.SHORT);
        else if (this.state == State.SHORT) setState(State.BOTTOM);
    }

    public boolean atBottom() {
        return this.state == State.BOTTOM;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("slide ticks: ", slide.getPosition(MotorUnit.TICKS));
        telemetry.addData("time: ", Clock.elapsed(previousTime));
//        telemetry.addData("profile output: ", slide.followProfile(Clock.elapsed(previousTime)));

        switch (state) {
            case TOP:
                slide.setPosition(Constants.Arm.kSlideTop);
                telemetry.addData("state: ", "TOP");
                break;
            case CARRYING:
                slide.setPosition(Constants.Arm.kSlideCarry);
                telemetry.addData("state: ", "Carrying");
                break;
            case MIDDLE:
                slide.setPosition(Constants.Arm.kSlideMiddle);
                telemetry.addData("state: ", "Middle");
                break;
            case TopOfMiddle:
                slide.setPosition(Constants.Arm.kSlideMiddle-600);
                break;
            case BOTTOM:
                slide.setPosition(Constants.Arm.kSlideBottom);
                telemetry.addData("state: ", "BOTTOM");
                break;
            case SHORT:
                slide.setPosition(Constants.Arm.kSlideShort);
                break;
            case TopOfShort:
                slide.setPosition(Constants.Arm.kSlideShort-400);
                break;
        }
    }
}
