package org.firstinspires.ftc.teamcode.teleop.opModes.test;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teleop.FSM;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@TeleOp(name="FSMTest", group="test")
public class FSMTest extends OpMode {
    // Standard Robot Classes
    private Robot robot;
    private GamepadMapping controls;
    private FSM fsm;

    // Pedro Drivetrain
    private Follower follower;

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.startingPose);
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        fsm.update();

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x * 0.55;

        follower.setTeleOpDrive(forward, strafe, rotate, true);

        telemetry.addData("Control Type", fsm.getControlType());
        telemetry.addData("Switch value", controls.switchMode.value());
    }

    @Override
    public void stop() {
        PoseStorage.startingPose = follower.getPose();
    }
}
