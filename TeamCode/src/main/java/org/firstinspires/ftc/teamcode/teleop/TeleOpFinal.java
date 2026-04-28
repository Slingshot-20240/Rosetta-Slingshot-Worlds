package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teleop.FSM;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@Config
@TeleOp(name="ASling TeleOp Final")
public class TeleOpFinal extends OpMode {
    private Robot robot;
    private GamepadMapping controls;
    private FSM fsm;

    private Follower follower;
    private Shooter shooter;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot    = new Robot(hardwareMap, controls);
        fsm      = new FSM(hardwareMap, controls, robot);
        shooter  = new Shooter(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.startingPose);
        follower.update();
    }

    @Override
    public void start() {
        Robot.limelight.start();
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        fsm.update();

        // Keep limelight fresh every loop
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        Robot.limelight.update(orientation);

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x * 0.55;

        if (gamepad1.a) {
            // Auto-align overrides rotate; follower handles axial/lateral
            boolean aligned = Robot.limelight.autoAlign(forward, strafe);

            // Still update follower with 0 rotate so Pedro doesn't fight the limelight
            follower.setTeleOpDrive(forward, strafe, 0, true);

            telemetry.addLine(">> AUTO ALIGNING (hold A)");
            telemetry.addData("aligned",           aligned);
            telemetry.addData("bearing (tX)",      Robot.limelight.getAngleBearing());
            telemetry.addData("distance (inches)", Robot.limelight.getDistanceInches());
            telemetry.addData("has target",        Robot.limelight.hasTarget());
        } else {
            // Full driver control via Pedro follower
            follower.setTeleOpDrive(forward, strafe, rotate, true);

            telemetry.addLine(">> DRIVER CONTROL (hold A to align)");
            telemetry.addData("has target",        Robot.limelight.hasTarget());
            telemetry.addData("bearing (tX)",      Robot.limelight.getAngleBearing());
            telemetry.addData("distance (inches)", Robot.limelight.getDistanceInches());
        }

        telemetry.addData("Control Type", fsm.getControlType());
        telemetry.addData("Switch value", controls.switchMode.value());
        telemetry.update();
    }

    @Override
    public void stop() {
        Robot.limelight.stop();
        PoseStorage.startingPose = follower.getPose();
    }
}