package org.firstinspires.ftc.teamcode.shplib.hardware.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class SHPTankDrive extends SHPFourWheelDrive {

    public SHPTankDrive(HardwareMap hardwareMap, String[] motorNames) {
        super(hardwareMap, motorNames);
    }

    public void tank(double leftPower, double rightPower) {
        double[] powers = {
                leftPower,
                leftPower,
                rightPower,
                rightPower
        };

        setAll(powers);
    }

    public void arcade(double straight, double turn) {
        double[] powers = {
                straight + turn,
                straight + turn,
                straight - turn,
                straight - turn
        };

        setAll(powers);
    }
}
