package org.firstinspires.ftc.teamcode.shplib.controllers;

import org.firstinspires.ftc.teamcode.Constants;

public class PositionPID extends PIDController {
    private double currentPosition;

    public PositionPID(double kP, double currentPosition) {
        super(kP * Constants.K_POSITION_PID_FACTOR);
        setCurrentPosition(currentPosition);
    }

    public PositionPID(double kP, double kI, double kD, double currentPosition) {
        super(kP * Constants.K_POSITION_PID_FACTOR, kI * Constants.K_POSITION_PID_FACTOR, kD * Constants.K_POSITION_PID_FACTOR);
        setCurrentPosition(currentPosition);
    }

    public void setCurrentPosition(double currentPosition) {
        this.currentPosition = currentPosition;
    }

    public double calculate(double desiredPosition) {
        return super.calculate(currentPosition, desiredPosition);
    }

    /**
     * Only use if you want to provide a different currentPosition
     *
     * @param currentPosition
     * @param desiredPosition
     * @return
     */
    @Override
    public double calculate(double currentPosition, double desiredPosition) {
        return super.calculate(currentPosition, desiredPosition);
    }
}
