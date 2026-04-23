package org.firstinspires.ftc.teamcode.NextFTC.autonomous.alliance;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.s;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
//import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.BaseShooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Stoppernf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "leaveAuto")
public class leaveAuto extends NextFTCOpMode {
    public leaveAuto() {
        addComponents(
                new SubsystemComponent(
                        f.i, s.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Stoppernf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    public PathChain path;


    public void buildPaths() {
        follower().setStartingPose(new Pose(88, 8.2, Math.toRadians(0)));

        path = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(88, 8.2),

                                new Pose(125, 8.2)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

    }

    private Command autonomous() {
        return new SequentialGroup(
                new FollowPath(path)
        );
    }


    @Override
    public void onInit() {
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        autonomous().schedule();
    }

    @Override
    public void onStop() {
        PoseStorage.startingPose = follower().getPose();
    }
}

