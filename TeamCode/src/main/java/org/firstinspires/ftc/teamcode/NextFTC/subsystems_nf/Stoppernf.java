package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Stoppernf implements Subsystem {
    public static final Stoppernf INSTANCE = new Stoppernf();
    private Stoppernf() {}

    public ServoEx stopper;

    public Command open() {
        return new SetPosition(stopper,0.55).requires(this);
    }

    public Command close() {
        return new SetPosition(stopper,0.36).requires(this);
    }

    @Override
    public void initialize() {
        stopper = new ServoEx("stopper");
    }

    @Override
    public void periodic() {}
}