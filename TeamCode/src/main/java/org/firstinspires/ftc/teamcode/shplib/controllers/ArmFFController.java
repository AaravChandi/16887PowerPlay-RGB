package org.firstinspires.ftc.teamcode.shplib.controllers;

public class ArmFFController extends FFController {

    public ArmFFController(double kV) {
        super(kV);
    }

    public ArmFFController(double kV, double kS) {
        super(kV, kS);
    }

    public ArmFFController(double kV, double kA, double kS) {
        super(kV, kA, kS);
    }
}
