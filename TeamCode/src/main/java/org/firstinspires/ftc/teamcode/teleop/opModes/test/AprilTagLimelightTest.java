package org.firstinspires.ftc.teamcode.teleop.opModes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@Config
@TeleOp(name="Limelight Test", group="test")
public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;
    private Robot robot;
    private GamepadMapping controls;


    double kP = 0.002;
    double error = 0;
    double lastError = 0;
    double goalX = 0; //offset here
    double angleTolerance = 0.4;
    double kD = 0.0001;
    double curTime = 0;
    double lastTime = 0;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // Both goal tags
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double tX = -llResult.getTx();
//            double tY = llResult.getTy();
            double tA = llResult.getTa();
            double dist = 2465.489 / tA;
//
            telemetry.addData("tX (bearing) ",tX);
//            telemetry.addData("tY",tY);
            telemetry.addData("tA", tA);
            telemetry.addData("distance (formula)", dist);
//            telemetry.addData("distance (botpose)", llResult.getBotposeAvgDist());

        } else {
            telemetry.addLine("No tag found");
        }

        controls.joystickUpdate();
        robot.drivetrain.update();
    }
}
