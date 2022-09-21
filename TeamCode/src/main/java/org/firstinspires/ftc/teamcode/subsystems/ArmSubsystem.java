package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

// Use this class as a reference for creating new subsystems

public class ArmSubsystem extends Subsystem {
    public final SHPMotor slide;
    public final SHPMotor actuator;

    public enum State {
        TOP,
        MIDDLE,
        BOTTOM,
    }

    private State state;

    private double previousTime;

    public ArmSubsystem(HardwareMap hardwareMap) {
        slide = new SHPMotor(hardwareMap, Constants.Arm.kSlideName, MotorUnit.ROTATIONS);
        slide.enablePositionPID(Constants.Arm.kSlideP);
        slide.setPositionErrorTolerance(Constants.Arm.kSlideTolerance);
//        slide.enableVelocityPID(Constants.Arm.kSlideP);
//        slide.enableProfiling(Constants.Arm.kSlideMaxVelocity);


        actuator = new SHPMotor(hardwareMap, Constants.Arm.kActuatorName);
        actuator.enablePositionPID(Constants.Arm.kActuatorP);
        actuator.setPositionErrorTolerance(Constants.Arm.kActuatorTolerance);

        previousTime = Clock.now();
        setState(State.BOTTOM);
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
        else if (this.state == State.BOTTOM) setState(State.MIDDLE);
    }

    public void previousState() {
        if (this.state == State.TOP) setState(State.MIDDLE);
        else if (this.state == State.MIDDLE) setState(State.BOTTOM);
    }

    public boolean atSetpoint() {
        return actuator.atPositionSetpoint();
    }

    public boolean atBottom() {
        return this.state == State.BOTTOM;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("slide rotations: ", slide.getPosition(MotorUnit.ROTATIONS));
        telemetry.addData("actuator enc: ", actuator.getPosition(MotorUnit.TICKS));
        telemetry.addData("arm at setpoint: ", atSetpoint() ? "true" : "false");
        telemetry.addData("time: ", Clock.elapsed(previousTime));
//        telemetry.addData("profile output: ", slide.followProfile(Clock.elapsed(previousTime)));

        switch (state) {
            case TOP:
                slide.setPosition(Constants.Arm.kSlideTop);
                actuator.setPosition(Constants.Arm.kActuatorTop);
                telemetry.addData("state: ", "TOP");
                break;
            case MIDDLE:
                slide.setPosition(Constants.Arm.kSlideMiddle);
                actuator.setPosition(Constants.Arm.kActuatorMiddle);
                telemetry.addData("state: ", "MIDDLE");
                break;
            case BOTTOM:
                slide.setPosition(Constants.Arm.kSlideBottom);
                actuator.setPosition(Constants.Arm.kActuatorBottom);
                telemetry.addData("state: ", "BOTTOM");
                break;
        }
    }
}
