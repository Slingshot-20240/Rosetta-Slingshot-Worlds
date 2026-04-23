package org.firstinspires.ftc.teamcode.NextFTC.autonomous.misc;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.s;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.BaseShooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Stoppernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;


import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@Autonomous(name = "zSimple Auton Test")
public class SimpleAutonTest extends NextFTCOpMode {
    public SimpleAutonTest() {
        addComponents(
                new SubsystemComponent(
                        f.i, s.i,
                        Intakenf.INSTANCE,
                        Stoppernf.INSTANCE,
                        Hoodnf.INSTANCE,
                        BaseShooternf.INSTANCE
//                        Lednf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }



    public PathChain scorePreloads_grabSet2;

    public PathChain scoreSet2;

    public Pose scorePose = new Pose(87,87);

    public void buildPaths() {

        PedroComponent.follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));

        scorePreloads_grabSet2 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(126.2, 119),
                                //TODO - try working with increasing x below, this makes it not go as far left for shooting
                                // position. This means lower shooter speed and make hood lower for more arc
                                new Pose(55.400, 76.100),
                                new Pose(103.389, 85.000),
                                new Pose(126, 83.000)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                        new HeadingInterpolator.PiecewiseNode(
                                                0,
                                                0.1,
                                                HeadingInterpolator.constant(Math.toRadians(36))
                                        ),
                                        new HeadingInterpolator.PiecewiseNode(
                                                0.1,
                                                0.45,
                                                HeadingInterpolator.facingPoint(141.5, 141.5)
                                        ),
                                        new HeadingInterpolator.PiecewiseNode(
                                                0.45,
                                                1.0,
                                        HeadingInterpolator.constant(0)
//                                                HeadingInterpolator.linearFromPoint(
//                                                        //TODO - whatever current pose is, not 45 degrees hardcoded
//                                                        () -> follower().getHeading(),
//                                                        Math.toRadians(0),
//                                                        0.995
//                                                )
                                        )
                                )
                )
                .addParametricCallback(0.25, () -> follower().setMaxPower(0.35))
                .addParametricCallback(0.55, () -> follower().setMaxPower(1.0))

                .build();



        scoreSet2 = follower().pathBuilder()
                //Score Set 2
                .addPath(
                        new BezierLine(new Pose(126, 83), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                //was 0
                .build();



    }


    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.3),
                Stoppernf.INSTANCE.close()
        );

    }


    private Command autonomous() {
        return new SequentialGroup(

                new ParallelDeadlineGroup(
                        new FollowPath(scorePreloads_grabSet2),

                        Intakenf.INSTANCE.in(),
                        s.i.shooterState(1000,0.3),
                        new SequentialGroup(
                                new WaitUntil(() -> PedroComponent.follower().getCurrentTValue() >= 0.35),
                                Stoppernf.INSTANCE.open(),
                                new Delay(1),
                                Stoppernf.INSTANCE.close()
                        )
                ),
//
                new ParallelDeadlineGroup(
                        new FollowPath(scoreSet2),

                        s.i.shootSequence(scoreSet2,2)

                )


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
        BaseShooternf.INSTANCE.enable();
        autonomous().schedule();
    }

    @Override
    public void onUpdate() {
    }


    @Override
    public void onStop() {
        PoseStorage.startingPose = PedroComponent.follower().getPose();
    }


}