package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Stopper {
    public final Servo stopper;

    public Stopper(HardwareMap hardwareMap) {
        stopper = hardwareMap.get(Servo.class, "stopper");
    }

    public Stopper(Servo stopper) {
        this.stopper = stopper;
    }
// ------------------------------------------------------------------

    public void release() {
        stopper.setPosition(0.48);
    }
    public void stop() {
        stopper.setPosition(0.7);
    }
}

