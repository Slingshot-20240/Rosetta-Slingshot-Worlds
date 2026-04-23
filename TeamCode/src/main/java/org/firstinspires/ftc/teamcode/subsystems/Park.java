package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Park {
    public Servo parkServo1;
    public Servo parkServo2;

    public Park(HardwareMap hwMap) {
        parkServo1 = hwMap.get(Servo.class, "park1");
        parkServo2 = hwMap.get(Servo.class, "park2");
    }

    public Park(Servo park1, Servo park2) {
        parkServo1 = park1;
        parkServo2 = park2;
    }

    public void tilt() {
        parkServo1.setPosition(0.22);
        parkServo2.setPosition(0.22);
    }

    public void unTilt() {
        parkServo1.setPosition(0.53);
        parkServo2.setPosition(0.53);
    }

}
