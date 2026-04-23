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
    DcMotorEx flywheel1;
    DcMotorEx flywheel2;
    //public static double p1 = 150, i1 = 0.0, d1 = 0.0, f1 = 20;
    public static double p1 = 185, i1 = 0.0, d1 = 0.0, f1 = 32;
    public static int targetVel = 1095;
    private Telemetry dashboardTelemetry;
    public static double hoodPos = 0.1;
    Robot robot;
    GamepadMapping controls;
    Drivetrain drivetrain;

    @Override
    public void init() {
        double time = 0;
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        flywheel1 = hardwareMap.get(DcMotorEx.class, "outtakeTop");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "outtakeBot");
 
        // Set PIDF (start with defaults, tune later)
        flywheel1.setVelocityPIDFCoefficients(578, 0, 0, 70);
        flywheel2.setVelocityPIDFCoefficients(578, 0, 0, 70);
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        drivetrain = new Drivetrain(hardwareMap, robot.imu, controls);
    }

    @Override
    public void loop() {
        //time += 0.02; // ~50 Hz loop

        // Updates driver controls here as well
        drivetrain.update();
        // Updates all other controls
        controls.update();

        robot.intake.intakeTransferOnClose();

//        if (controls.transfer.value()) {
//            robot.stopper.close();
//        } else {
//            robot.stopper.open();
//        }

//        if (controls.outtake.locked()) {
//            robot.intake.intakeTransferReverse();
//        } else {
//            robot.intake.intakeTransferOn();
//        }

        //sine wave/variable setpoint between 2000 and 5000 ticks/sec
        //double targetVel = 3500 + 1500 * Math.sin(2 * Math.PI * 0.5 * time);

      // Send target to REV Hub PID
        flywheel1.setVelocity(targetVel);
        flywheel2.setVelocity(targetVel);
        flywheel1.setVelocityPIDFCoefficients(p1, i1, d1, f1);
        flywheel2.setVelocityPIDFCoefficients(p1, i1, d1, f1);

        //robot.shooter.variableHood.setPosition(hoodPos);

        // Read actual velocity
        double actualVel1 = flywheel1.getVelocity();
        double actualVel2 = flywheel2.getVelocity();


        // Telemetry
        dashboardTelemetry.addData("Target (ticks/s): ", targetVel);
        dashboardTelemetry.addData("Actual1 (ticks/s): ", actualVel1);
        dashboardTelemetry.addData("Actual2 (ticks/s): ", actualVel2);
        dashboardTelemetry.addData("Encoder1:", flywheel1.getCurrentPosition());
        dashboardTelemetry.addData("Encoder2:", flywheel2.getCurrentPosition());
        dashboardTelemetry.update();

        // sleep(20);
    }
}

