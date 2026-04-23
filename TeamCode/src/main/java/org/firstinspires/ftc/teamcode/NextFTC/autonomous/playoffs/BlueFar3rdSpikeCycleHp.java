package org.firstinspires.ftc.teamcode.NextFTC.autonomous.playoffs;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

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
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "Blue 3rd spike far")
public class BlueFar3rdSpikeCycleHp extends NextFTCOpMode {
    public BlueFar3rdSpikeCycleHp() {
        addComponents(
                new SubsystemComponent(
                        f.i, s.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        BaseShooternf.INSTANCE, Stoppernf.INSTANCE
//                        Lednf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads;
    public PathChain grabDownCycle;
    public PathChain grabSet2, scoreSet2;
    public PathChain scoreDownCycle;
    public PathChain grabUpCycle;
    public PathChain scoreUpCycle;
    public PathChain park;
    public double scoreHeading = 180-69;

    public Pose scorePose = new Pose(88,15).mirror();

    public void buildPaths() {
        follower().setStartingPose(new Pose(88, 8.2, Math.toRadians(90)).mirror());

        scorePreloads = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(88, 8.2).mirror(),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-90), Math.toRadians(scoreHeading))

                .build();


        grabSet2 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(91.389, 37.241).mirror(),
                                new Pose(133.000, 36.000).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(scoreHeading), Math.toRadians(180-0))

                .build();

        scoreSet2 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133.000, 36.000).mirror(),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(scoreHeading))

                .build();

        grabDownCycle = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(106.000, 11.000).mirror(),
                                new Pose(134.000, 10.000).mirror()
                        )
                ).setTangentHeadingInterpolation()

                .build();

        scoreDownCycle = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.000, 10.000).mirror(),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(scoreHeading))

                .build();

        grabUpCycle = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(110.593, 22.685).mirror(),
                                new Pose(133.000, 22.000).mirror()
                        )
                ).setTangentHeadingInterpolation()

                .build();

        scoreUpCycle = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133.000, 22.000).mirror(),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(scoreHeading))

                .build();

        park = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(120.000, 15.000).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(scoreHeading), Math.toRadians(180-90))

                .build();



    }

    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.32),
                Stoppernf.INSTANCE.close()
        );

    }

    private Command autonomous() {
        return new SequentialGroup(

                //Score Preloads
                new ParallelGroup(
                        new FollowPath(scorePreloads),

                        Intakenf.INSTANCE.in(),
                        BaseShooternf.INSTANCE.setShooterVel(1300)
                ),
                new Delay(1.5),
                s.i.shoot(2, 0.6),

                //Set 2
                new FollowPath(grabSet2),
                new FollowPath(scoreSet2),
                s.i.shoot(1.5,0.6),

                //Set 3
                new FollowPath(grabDownCycle),
                new FollowPath(scoreDownCycle),
                s.i.shoot(1.5,0.6),

                //Set 4
                new FollowPath(grabUpCycle),
                new FollowPath(scoreUpCycle),
                new Delay(0.1),
                s.i.shoot(1.5,0.6),

                //Set 5
                new FollowPath(grabDownCycle),
                new FollowPath(scoreDownCycle),
                s.i.shoot(1.5,0.6),

                new FollowPath(park)


        );
    }


    @Override
    public void onInit() {
        buildPaths();
        init_bot().schedule();
        BaseShooternf.INSTANCE.disable();
    }

    @Override
    public void onStartButtonPressed() {
        autonomous().schedule();
        BaseShooternf.INSTANCE.enable();
    }

    @Override
    public void onStop() {
        PoseStorage.startingPose = follower().getPose();
    }
}