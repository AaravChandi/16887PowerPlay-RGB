package org.firstinspires.ftc.teamcode.shplib.hardware.drive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

public class SHPFourWheelDrive {
    public final SHPMotor[] motors;

    public SHPFourWheelDrive(HardwareMap hardwareMap, String[] motorNames) {
        // 0 -> left front
        // 1 -> left rear
        // 2 -> right front
        // 3 -> right rear
        motors = new SHPMotor[4];
        for (int i = 0; i < motors.length; i++) {
            motors[i] = new SHPMotor(hardwareMap, motorNames[i]);
        }
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void enablePositionPID(double kP) {
        for (SHPMotor motor : motors) {
            motor.enablePositionPID(kP);
        }
    }

    public void enablePositionPID(double kP, double kI, double kD) {
        for (SHPMotor motor : motors) {
            motor.enablePositionPID(kP, kI, kD);
        }
    }

    public void enableVelocityPID(double kP) {
        for (SHPMotor motor : motors) {
            motor.enableVelocityPID(kP);
        }
    }

    public void enableVelocityPID(double kP, double kI, double kD) {
        for (SHPMotor motor : motors) {
            motor.enableVelocityPID(kP, kI, kD);
        }
    }

    public void setAll(double[] powers) {
        for (int i = 0; i < motors.length; i++) {
            setPower(i, powers[i]);
        }
    }

    public void setPower(int index, double power) {
        motors[index].setPower(power);
    }

    public void reverseAll() {
        for (SHPMotor motor : motors) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
}
