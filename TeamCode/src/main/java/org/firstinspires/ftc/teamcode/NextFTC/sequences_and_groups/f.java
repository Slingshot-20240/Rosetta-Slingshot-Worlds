package org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups;


import com.pedropathing.paths.PathChain;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.extensions.pedro.FollowPath;

public class f extends SubsystemGroup {
    public static final f i = new f();

    private f() {
        super(/*insert subsystem instance here*/);
    }

    public final Command follow(PathChain path) {
        return new SequentialGroup(
                new FollowPath(path)
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

}