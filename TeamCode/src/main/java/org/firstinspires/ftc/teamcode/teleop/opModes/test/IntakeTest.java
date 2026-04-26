package org.firstinspires.ftc.teamcode.teleop.opModes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@Config
@TeleOp (name = "Intake Test", group = "tests")
public class IntakeTest extends OpMode {

    public static double power = 1.0;
    public static double position = 0.67;
    private Intake intake;
    DcMotor leftFront, rightFront;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");


        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
//        intake.setPivotPos(position);
        intake.setIntakePower(power);
        leftFront.setPower(1.0);
        rightFront.setPower(1.0);
    }
}
