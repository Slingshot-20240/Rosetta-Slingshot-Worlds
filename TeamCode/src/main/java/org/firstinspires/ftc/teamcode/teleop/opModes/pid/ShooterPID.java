package org.firstinspires.ftc.teamcode.teleop.opModes.pid;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;


@Config
@TeleOp(name = "ShooterPID", group = "Testing")
public class ShooterPID extends OpMode {
//test
    DcMotorEx outtake1;
    DcMotorEx outtake2;
    //public static double p1 = 150, i1 = 0.0, d1 = 0.0, f1 = 20;
    public static double p1 = 185, i1 = 0.0, d1 = 0.0, f1 = 32;
    public static int targetVel = 1095;
    public static double hoodAngle = 0.5;
    private Telemetry dashboardTelemetry;
    Robot robot;
    GamepadMapping controls;
    Drivetrain drivetrain;

    @Override
    public void init() {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtake1 = hardwareMap.get(DcMotorEx.class, "ol");
        outtake2 = hardwareMap.get(DcMotorEx.class, "or");
 
        // Set PIDF (start with defaults, tune later)
        outtake1.setVelocityPIDFCoefficients(578, 0, 0, 70);
        outtake2.setVelocityPIDFCoefficients(578, 0, 0, 70);
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);

        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        drivetrain = new Drivetrain(hardwareMap, robot.imu, controls);
    }

    @Override
    public void loop() {
        drivetrain.update();
        controls.update();
        robot.intake.setIntakePower(1);

        outtake1.setVelocity(targetVel);
        outtake2.setVelocity(targetVel);
        outtake1.setVelocityPIDFCoefficients(p1, i1, d1, f1);
        outtake2.setVelocityPIDFCoefficients(p1, i1, d1, f1);
        robot.shooter.variableHood.setPosition(hoodAngle);



        // Read actual velocity
        double actualVel1 = outtake1.getVelocity();
        double actualVel2 = outtake2.getVelocity();


        // Telemetry
        dashboardTelemetry.addData("Target (ticks/s): ", targetVel);
        dashboardTelemetry.addData("Actual1 (ticks/s): ", actualVel1);
        dashboardTelemetry.addData("Actual2 (ticks/s): ", actualVel2);
        dashboardTelemetry.addData("Encoder1:", outtake1.getCurrentPosition());
        dashboardTelemetry.addData("Encoder2:", outtake2.getCurrentPosition());
        dashboardTelemetry.update();


    }
}

