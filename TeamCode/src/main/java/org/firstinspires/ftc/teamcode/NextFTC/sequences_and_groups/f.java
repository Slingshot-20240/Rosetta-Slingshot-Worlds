package org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups;


import com.pedropathing.paths.PathChain;

//import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.misc.vision.Logi;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.TurnBy;

public class f extends SubsystemGroup {
    public static final f i = new f();

    private f() {
        super(
//                Lednf.INSTANCE
        );
    }

    public final Command follow(PathChain path) {
        return new SequentialGroup(
                new FollowPath(path)
        );
    }
    public final Command follow(PathChain path, String color) {
        return new ParallelGroup(
                new FollowPath(path)
//                Lednf.INSTANCE.color(color)
        );
    }
    public final Command follow(PathChain path, String color, boolean holdEnd) {
        return new ParallelGroup(
                new FollowPath(path, holdEnd)
//                Lednf.INSTANCE.color(color)
        );
    }
    public final Command follow(PathChain path, String color, boolean holdEnd, double maxPower) {
        return new ParallelGroup(
                new FollowPath(path, holdEnd, maxPower)
//                Lednf.INSTANCE.color(color)
        );
    }
    public final Command follow(PathChain path, boolean holdEnd) {
        return new SequentialGroup(
                new FollowPath(path, holdEnd)
        );
    }
    public final Command follow(PathChain path, boolean holdEnd, double maxPower) {
        return new SequentialGroup(
                new FollowPath(path, holdEnd, maxPower)
        );
    }

    public final Command autoAlign() {
        return new SequentialGroup(
                new TurnBy(Angle.fromDeg(Logi.INSTANCE.getATangle()))
        );
    }



}