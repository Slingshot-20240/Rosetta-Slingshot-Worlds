package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@Config
@TeleOp(name = "Limelight Test", group = "test")
public class AutoAlignTest_Ishaan extends OpMode {

    private Limelight3A limelight;
    private Robot robot;
    private GamepadMapping controls;

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    public static double kP             = 0.02;
    public static double kD             = 0.001;
    public static double goalX          = 0.0;
    public static double angleTolerance = 0.4;

    private double error     = 0;
    private double lastError = 0;
    private double lastTime  = 0;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot    = new Robot(hardwareMap, controls);

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront .setDirection(DcMotor.Direction.REVERSE);
        leftBack  .setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack .setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
    }

    @Override
    public void start() {
        limelight.start();
        lastTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            double tX   = -llResult.getTx();
            double tA   =  llResult.getTa();
            double dist = 30665.95 / tA;

            double curTime = System.currentTimeMillis();
            double dt      = (curTime - lastTime) / 1000.0;
            lastTime = curTime;

            error = tX - goalX;

            double derivative = (dt > 0) ? (error - lastError) / dt : 0;
            double yawPower   = kP * error + kD * derivative;
            lastError = error;

            if (Math.abs(error) > angleTolerance) {
                mecanumDrive(0, 0, yawPower);
            } else {
                mecanumDrive(0, 0, 0);
            }

            telemetry.addData("tX (bearing)", tX);
            telemetry.addData("tA",           tA);
            telemetry.addData("distance (formula)", dist);
        } else {
            mecanumDrive(0, 0, 0);
            telemetry.addLine("No tag found");
        }

        telemetry.update();
    }

    private void mecanumDrive(double axial, double lateral, double yaw) {
        double lf = axial + lateral + yaw;
        double rf = axial - lateral - yaw;
        double lb = axial - lateral + yaw;
        double rb = axial + lateral - yaw;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        leftFront .setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack  .setPower(lb / max);
        rightBack .setPower(rb / max);
    }
}