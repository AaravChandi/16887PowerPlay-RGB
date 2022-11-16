package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

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
        TopOfTop,
        MIDDLE,
        BOTTOM,
        SHORT,
        TopOfShort,
        TopOfMiddle,
        CARRYING,
        StackedCones,
        MANUAL,
    }

    private State state;

    private double previousTime;

    public ArmSubsystem(HardwareMap hardwareMap) {
        slide = new SHPMotor(hardwareMap, Constants.Arm.kSlideName, MotorUnit.TICKS);
        slide.reverseDirection();
        slide.enablePositionPID(Constants.Arm.kSlideP);
        slide.setPositionErrorTolerance(Constants.Arm.kSlideTolerance);
        slide.enableFF(new ElevatorFFController(0, Constants.Arm.kSlideG));
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
        if (state == State.StackedCones)
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
        if (this.state == State.TOP) setState(State.TopOfTop);
        else if (this.state == State.MIDDLE) setState(State.TopOfMiddle);
        else if (this.state == State.SHORT) setState(State.TopOfShort);
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
                    slide.setPosition(Constants.Arm.kSlideMiddle - 600);
                    break;
                case BOTTOM:
                    slide.setPosition(Constants.Arm.kSlideBottom);
                    telemetry.addData("state: ", "BOTTOM");
                    break;
                case SHORT:
                    slide.setPosition(Constants.Arm.kSlideShort);
                    break;
                case TopOfShort:
                    slide.setPosition(Constants.Arm.kSlideShort - 400);
                    break;
                case TopOfTop:
                    slide.setPosition(Constants.Arm.kSlideTop - 500);
                    break;
                case StackedCones:
                    slide.setPosition(Constants.Arm.kSlideBottom + coneLevel*200);
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
