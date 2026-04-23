package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intakenf implements Subsystem {

    public static final Intakenf INSTANCE = new Intakenf();

    private Intakenf() { }

    public MotorEx intake1;
    public MotorEx dropDownIntake;
    MotorGroup intake;



    public Command in() {
        return new SetPower(intake, 0.9);
    }
    public Command idle() {
        return new SetPower(intake, 0);
    }
    public Command out() {
        return new SetPower(intake, -1.0);
    }

    public Command setIntakePower(double power) {
        return new SetPower(intake, power);
    }


    @Override
    public void initialize() {
        intake1 = new MotorEx("intake1");
        dropDownIntake = new MotorEx("dropdownIntake");
        intake1.reverse();
        dropDownIntake.reverse();
        intake = new MotorGroup(intake1, dropDownIntake);
    }

    @Override
    public void periodic() {}

}
