package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;

public class Turretnf implements Subsystem {
    public static final Turretnf INSTANCE = new Turretnf();
    private Turretnf() {}

    public ServoEx leftTurret;
    public ServoEx rightTurret;


    public Command setPosition(double position) {
        return new ParallelGroup(
                new SetPosition(leftTurret, position),
                new SetPosition(rightTurret, position)
        );
    }

    @Override
    public void initialize() {
        leftTurret = new ServoEx("lt");
        rightTurret = new ServoEx("rt");
    }

    @Override
    public void periodic() {}
}