package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class FarShooternf implements Subsystem {
    public static final FarShooternf INSTANCE = new FarShooternf();
    private FarShooternf() { }

    public DcMotorEx outtake1, outtake2;
    private boolean enabled = false;

    public Command setShooterVel(double shooterVel) {
        return new InstantCommand(
                () -> {
                    outtake1.setVelocity(shooterVel);
                    outtake2.setVelocity(shooterVel);
                }
        );
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        }

    @Override
    public void initialize() {
        outtake1 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "outtakeTop");
        outtake2 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "outtakeBot");
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setVelocityPIDFCoefficients(185,0,0,32);
        outtake2.setVelocityPIDFCoefficients(185,0,0,32);

        disable();
    }

    @Override
    public void periodic() {
        if (!enabled) {
            outtake1.setPower(0);
            outtake2.setPower(0);
            return;
        }


    }
}
