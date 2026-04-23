package org.firstinspires.ftc.teamcode.teleop.opModes.test.servoTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@Config
@TeleOp(group = "tests")
public class StopperTest extends OpMode {
    Robot robot;
    public static double stopperPos = 0.6; // release 0.48
    public static double stage2Speed = 0;
    GamepadMapping controls;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
    }

    @Override
    public void loop() {
        robot.stopper.stopper.setPosition(stopperPos);

        robot.intake.intake1.setPower(stage2Speed);

        robot.intake.dropdownIntake.setPower(-1);

        // 0.55 release
        // 0.6 stop
    }
}
