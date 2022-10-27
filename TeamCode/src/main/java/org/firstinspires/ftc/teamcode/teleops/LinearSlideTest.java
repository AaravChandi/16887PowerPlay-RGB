package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.BaseRobotLinearSlide;
import org.firstinspires.ftc.teamcode.Constants;
//import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

@TeleOp
public class LinearSlideTest extends BaseRobotLinearSlide {
    //public SHPMotor slide;

    @Override
    public void init() {
        super.init();
        /*slide = new SHPMotor(hardwareMap, Constants.Arm.kSlideName, MotorUnit.TICKS);
        slide.enablePositionPID(Constants.Arm.kSlideP);
        slide.setPositionErrorTolerance(Constants.Arm.kSlideTolerance);*/

        // Default command runs when no other commands are scheduled for the subsystem
        /*drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );*/
    }

    @Override
    public void start() {
        super.start();
        //slide.resetEncoder();


        // Add anything that needs to be run a single time when the OpMode starts
    }

    @Override
    public void loop() {

        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();
//        if (gamepad1.a) slide.setPosition(Constants.Arm.kSlideMiddle);//
//        telemetry.addData("slide: ", slide.getPosition(MotorUnit.TICKS));

        new Trigger(gamepad1.b, new MoveArmCommand(arm, MoveArmCommand.Direction.TOP)
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.MIDDLE))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
                //.then(new DumpCargoCommand(scoop))
                //.then(new MoveArmCommand(arm, MoveArmCommand.Direction.MIDDLE))
                //.then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
        );
        telemetry.addData("slide: ", arm.slide.getPosition(MotorUnit.TICKS));

        new Trigger(gamepad1.x,
                new RunCommand((() -> {
                    arm.setState(ArmSubsystem.State.TOP);}), arm)
                //        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))
                //        .then(new DumpCargoCommand(scoop))
                //.then(new MoveArmCommand(arm, MoveArmCommand.Direction.MIDDLE))
                //.then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
        );

    }

}
