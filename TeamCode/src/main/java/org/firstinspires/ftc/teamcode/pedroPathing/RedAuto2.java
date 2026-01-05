package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous(name = "RedAuto2", group = "Autonomous")
public class RedAuto2 extends OpMode {
    private Follower follower; //this guy just kinda executes the paths type stuff yk
    private Timer pathTimer, actionTimer, opmodeTimer; //Path timer can be used in the autonomousPathUpdate just to see if one of the paths failed or something
    private int pathState; //just an int used later in autonomousPathUpdate for each of the cases (tells which path to do)
    private final Pose startPose = new Pose(128, -128, Math.toRadians(-45)); // Start Pose of our robot. (I think these are the right measurements, as 0 degrees corresponds to facing right the starting x is a bit weird as it depends on where on the line we start)
    private final Pose scorePose1 = new Pose(96, -96, Math.toRadians(-45)); // Scoring Pose of our robot. (Random for right now idk where we will score)
    private final Pose intakePose1 = new Pose(83, -111, Math.toRadians(-90));//this is where we should intake the BALLS idk where it is at this time so change late
    private final Pose intakePose2 = new Pose(60, -111, Math.toRadians(-90));
    private final Pose endPose1 = new Pose(70, -115, Math.toRadians(0));
    private Path score1;
    private PathChain firstLoad, secondLoad, end, scoreLoad1, scoreLoad2;

    //10/17 - got auto working but want to add functionality where the robot corrects to stay in the same place when paused
    public void buildPaths() {//this is where we build the path stuff
        score1 = new Path(new BezierLine(startPose, scorePose1));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());
        firstLoad=follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, intakePose1))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intakePose1.getHeading())
                .build();
        scoreLoad1= follower.pathBuilder()
                .addPath(new BezierLine(intakePose1, scorePose1))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), scorePose1.getHeading())
                .build();
        secondLoad= follower.pathBuilder()
                .addPath(new BezierLine(scorePose1,intakePose2))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intakePose2.getHeading())
                .build();
        scoreLoad2= follower.pathBuilder()
                .addPath(new BezierLine(intakePose2, scorePose1))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), scorePose1.getHeading())
                .build();
        end= follower.pathBuilder()
                .addPath(new BezierLine(scorePose1,intakePose2))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), endPose1.getHeading())
                .build();

    }
    public void autonomousPathUpdate() throws InterruptedException {//we can add a lot more paths
        switch (pathState) {
            case 0:
                follower.followPath(score1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(firstLoad,true);
                    Thread.sleep(3000);
                   /* time = System.currentTimeMillis();
                    while (System.currentTimeMillis() < time + 10000){} */
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(scoreLoad1,true);
                    Thread.sleep(3000);
                   /* time = System.currentTimeMillis();
                    while (System.currentTimeMillis() < time + 10000){} */
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(secondLoad,true);
                    Thread.sleep(3000);
                    /*time = System.currentTimeMillis();
                    while (System.currentTimeMillis() < time + 10000){} */
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(scoreLoad2,true);
                    Thread.sleep(3000);
                    /*time = System.currentTimeMillis();
                    while (System.currentTimeMillis() < time + 1000){}*/
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(end,true);
                    Thread.sleep(3000);
                    /*time = System.currentTimeMillis();
                    while (System.currentTimeMillis() < time + 1000){}*/
                    setPathState(6);
                }
                break;

        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() { //this runs constantly during auto and we just update the position of the follower and check if it is still busy and cycle through each case
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}