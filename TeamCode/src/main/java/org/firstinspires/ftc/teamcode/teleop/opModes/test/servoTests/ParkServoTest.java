package org.firstinspires.ftc.teamcode.teleop.opModes.test.servoTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Park;

@TeleOp (group = "tests")
@Config
public class ParkServoTest extends OpMode {

    Park park;

    public static double position = 0.52;

    @Override
    public void init() {
        park = new Park(hardwareMap);
    }

    @Override
    public void loop() {
        park.parkServo1.setPosition(position);
        park.parkServo2.setPosition(position);
    }


}
