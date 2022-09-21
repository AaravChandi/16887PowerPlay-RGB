package org.firstinspires.ftc.teamcode.shplib.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.controllers.FFController;
import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
import org.firstinspires.ftc.teamcode.shplib.controllers.VelocityPID;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class SHPMotor {
    private final DcMotorEx motor;
    private VoltageSensor voltageSensor;

    private final MotorUnit unit;
    private double ticksPerRotation = 384.5; // default for gobilda 5202 @ 435 rpm
    private PositionPID positionPID;
    private VelocityPID velocityPID;
    private FFController ff;

    private boolean profilingEnabled = false;
    private double maxVelocity;
    private double maxAcceleration;
    private MotionProfile profile;

    public SHPMotor(@NonNull HardwareMap hardwareMap, String deviceName) {
        this.motor = hardwareMap.get(DcMotorEx.class, deviceName);
        this.unit = MotorUnit.TICKS;
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        enableVoltageCompensation(hardwareMap);
    }

    public SHPMotor(@NonNull HardwareMap hardwareMap, String deviceName, MotorUnit unit) {
        this.motor = hardwareMap.get(DcMotorEx.class, deviceName);
        this.unit = unit;
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        enableVoltageCompensation(hardwareMap);
    }

    public void enableVoltageCompensation(@NonNull HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void disableVoltageCompensation() {
        voltageSensor = null;
    }

    public void setTicksPerRotation(double ticksPerRotation) {
        this.ticksPerRotation = ticksPerRotation;
    }

    public void enablePositionPID(double kP) {
        positionPID = new PositionPID(kP, getPosition(unit));
    }

    public void enablePositionPID(double kP, double kI, double kD) {
        positionPID = new PositionPID(kP, kI, kD, getPosition(unit));
    }

    public void disablePositionPID() {
        positionPID = null;
    }

    public void setPositionErrorTolerance(double errorTolerance) {
        if (positionPID == null) return;
        positionPID.setErrorTolerance(errorTolerance);
    }

    public boolean atPositionSetpoint() {
        if (positionPID == null) return true;
        return positionPID.atSetpoint();
    }

    public void enableVelocityPID(double kP) {
        velocityPID = new VelocityPID(kP, getVelocity(unit));
    }

    public void enableVelocityPID(double kP, double kI, double kD) {
        velocityPID = new VelocityPID(kP, kI, kD, getVelocity(unit));
    }

    public void disableVelocityPID() {
        velocityPID = null;
    }

    public void setVelocityErrorTolerance(double errorTolerance) {
        if (velocityPID == null) return;
        velocityPID.setErrorTolerance(errorTolerance);
    }

    public boolean atVelocitySetpoint() {
        if (velocityPID == null) return true;
        return velocityPID.atSetpoint();
    }

    public void enableFF(FFController ff) {
        this.ff = ff;
    }

    public void disableFF() {
        ff = null;
    }

    /**
     * @param power -1.0 to 1.0
     */
    public void setPower(double power) {
        if (voltageSensor != null)
            power *= (Constants.kNominalVoltage / voltageSensor.getVoltage());
        power = Range.clip(power, -1.0, 1.0);
        if (getPower() != power)
            motor.setPower(power);
    }

    public double getPower() {
        return motor.getPower();
    }

    public double setVelocity(double velocity) {
        double power = 0.0;
        if (velocityPID != null) {
            velocityPID.setCurrentVelocity(getVelocity(unit));
            power += velocityPID.calculate(velocity);
        }
        if (ff != null) power += ff.calculate(velocity);
        setPower(power);
        return power;
    }

    public double getVelocity(MotorUnit unit) {
        if (unit == MotorUnit.DEGREES)
            return motor.getVelocity(AngleUnit.DEGREES);
        else if (unit == MotorUnit.RADIANS)
            return motor.getVelocity(AngleUnit.RADIANS);
        else if (unit == MotorUnit.ROTATIONS)
            return motor.getVelocity(AngleUnit.DEGREES) / 360.0;
        else
            return motor.getVelocity();
    }

    public double setPosition(double position) {
        if (positionPID == null) return 0.0;
        positionPID.setCurrentPosition(getPosition(unit));
        double power = positionPID.calculate(position);
        setPower(power);
        return power;
    }

    public double getPosition(MotorUnit unit) {
        if (unit == MotorUnit.DEGREES)
            return motor.getCurrentPosition() / ticksPerRotation * 360;
        else if (unit == MotorUnit.RADIANS)
            return Math.toRadians(motor.getCurrentPosition() / ticksPerRotation * 360);
        else if (unit == MotorUnit.ROTATIONS)
            return motor.getCurrentPosition() / ticksPerRotation;
        else
            return motor.getCurrentPosition();
    }

    public void enableProfiling(double maxVelocity) {
        enableProfiling(maxVelocity, maxVelocity);
    }

    public void enableProfiling(double maxVelocity, double maxAcceleration) {
        profilingEnabled = true;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public void disableProfiling() {
        profilingEnabled = false;
    }

    public void profileTo(double position) {
        if (!profilingEnabled) return;
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getPosition(unit), 0, 0),
                new MotionState(position, 0, 0),
                maxVelocity,
                maxAcceleration
        );
    }

    public double followProfile(double seconds) {
        if (!profilingEnabled || profile == null) return 0.0;
        MotionState state = profile.get(seconds);
        return setVelocity(state.getV());
    }

    public void setDirection(DcMotorEx.Direction direction) {
        motor.setDirection(direction);
    }

    public DcMotorSimple.Direction getDirection() {
        return motor.getDirection();
    }

    public void setMotorEnable() {
        motor.setMotorEnable();
    }

    public void setMotorDisable() {
        motor.setMotorDisable();
    }

    public boolean isMotorEnabled() {
        return motor.isMotorEnabled();
    }

    public boolean isMotorDisabled() {
        return !motor.isMotorEnabled();
    }

    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    public double getCurrentAlert(CurrentUnit unit) {
        return motor.getCurrentAlert(unit);
    }

    public void setCurrentAlert(double current, CurrentUnit unit) {
        motor.setCurrentAlert(current, unit);
    }

    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    public DcMotorController getController() {
        return motor.getController();
    }

    public int getPortNumber() {
        return motor.getPortNumber();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public DcMotor.RunMode getMode() {
        return motor.getMode();
    }

    public HardwareDevice.Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    public String getDeviceName() {
        return motor.getDeviceName();
    }

    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    public int getVersion() {
        return motor.getVersion();
    }

    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    public void resetEncoder() {
        resetEncoder(false);
    }

    public void resetEncoder(boolean runUsingBuiltInControl) {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (runUsingBuiltInControl) setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
