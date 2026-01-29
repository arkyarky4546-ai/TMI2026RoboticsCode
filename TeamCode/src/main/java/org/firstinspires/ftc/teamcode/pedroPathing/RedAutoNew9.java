package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intakeShoot;
@Autonomous(name = "RedAutoNew9", group = "Autonomous")
public class RedAutoNew9 extends OpMode {
    private Follower follower; //this guy just kinda executes the paths type stuff yk
    private Timer pathTimer, actionTimer, opmodeTimer; //Path timer can be used in the autonomousPathUpdate just to see if one of the paths failed or something

    //positions
    private int pathState; //just an int used later in autonomousPathUpdate for each of the cases (tells which path to do)
    private final Pose startPose = new Pose(128, -128, Math.toRadians(-47)); // Start Pose of our robot. (I think these are the right measurements, as 0 degrees corresponds to facing right the starting x is a bit weird as it depends on where on the line we start)
    private final Pose scorePose1 = new Pose(90, -92, Math.toRadians(-47)); // Scoring Pose of our robot. (Random for right now idk where we will score)
    private final Pose intakePose1 = new Pose(82, -96, Math.toRadians(-90));//this is where we should intake the BALLS idk where it is at this time so change late
    private final Pose acIntakePose1 = new Pose(82, -134 , Math.toRadians(-90));
    private final Pose intakePose2 = new Pose(59, -98, Math.toRadians(-90));
    private final Pose acIntakePose2 = new Pose(59, -137, Math.toRadians(-90));
    private final Pose endPose1 = new Pose(65, -97, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(92, -92, Math.toRadians(-50));

    //paths
    private Path score1;
    private PathChain firstLoad, secondLoad, acFirstLoad, acSecondLoad, end, scoreLoad1, scoreLoad2;

    //doubles
    double hoodPos = .25;
    double turTurn = .97;
    double kickZero = 0.85;
    double kickUp = 0.68;

    //timer
    private ElapsedTime shootTimer = new ElapsedTime();

    //custom class that controls shooting and intake
    private intakeShoot intakeAndShoot;

    //servos
    Servo push, turretRight, turretLeft, hood;

    //booleans
    boolean intakeIndex = true;
    //ints
    int index = 0;
    int shootPos = 1;
    int intakePos = 0;

    public void buildPaths() {//this is where we build the path stuff using our positions
        score1 = new Path(new BezierLine(startPose, scorePose1));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());
        firstLoad=follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, intakePose1))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intakePose1.getHeading())
                .build();
        acFirstLoad=follower.pathBuilder()
                .addPath(new BezierLine(intakePose1, acIntakePose1))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), acIntakePose1.getHeading())
                .build();
        scoreLoad1= follower.pathBuilder()
                .addPath(new BezierLine(acIntakePose1, scorePose1))
                .setLinearHeadingInterpolation(acIntakePose1.getHeading(), scorePose1.getHeading())
                .build();
        secondLoad= follower.pathBuilder()
                .addPath(new BezierLine(scorePose1,intakePose2))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intakePose2.getHeading())
                .build();
        acSecondLoad= follower.pathBuilder()
                .addPath(new BezierLine(intakePose2,acIntakePose2))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), acIntakePose2.getHeading())
                .build();
        scoreLoad2= follower.pathBuilder()
                .addPath(new BezierLine(acIntakePose2, scorePose1))
                .setLinearHeadingInterpolation(acIntakePose2.getHeading(), scorePose1.getHeading())
                .build();
        end= follower.pathBuilder()
                .addPath(new BezierLine(scorePose1,endPose1))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), endPose1.getHeading())
                .build();

    }
    public void autonomousPathUpdate() throws InterruptedException {//we can add a lot more paths
        switch (pathState) {

            case 0:
                //the follower is now on the score1 path
                follower.followPath(score1);
                intakeAndShoot.setPos(0,intakePos);
                intakeIndex = false;
                //go to the next case
                setPathState(1);

                break;
            case 1:
                if(!follower.isBusy()){
                    //reset action timer for holding the score position
                    actionTimer.resetTimer();
                    //method to hold a position
                    follower.holdPoint(scorePose1);
                    //open wall position
                    intakeAndShoot.wallPos(0);
                    shootTimer.reset();
                    setPathState(2);}
                break;
            case 2:
                if(shootTimer.milliseconds() > 900) {
                    push.setPosition(kickUp);
                    //shooting every 800 milliseconds
                    intakeAndShoot.simpleShoot();
                    //this is what I mean about the timer being used to delay stuff
                    shootTimer.reset();
                }
                if(actionTimer.getElapsedTimeSeconds() > 4.5) {
                    intakeAndShoot.setPos(0, intakePos);

                    follower.followPath(firstLoad,true);
                    //push servo is down now
                    push.setPosition(kickZero);
                    //closed wall position
                    intakeAndShoot.wallPos(.25);
                    intakeIndex = true;
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {

                    follower.followPath(acFirstLoad,.32,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.holdPoint(acIntakePose1);
                    actionTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:

                if(actionTimer.getElapsedTimeSeconds() > .1) {
                    intakeAndShoot.setPos(0,intakePos);
                    intakeAndShoot.wallPos(0);
                    intakeIndex = false;
                    follower.followPath(scoreLoad1,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {

                    actionTimer.resetTimer();
                    shootTimer.reset();
                    follower.holdPoint(scorePose1);
                    setPathState(7);
                }
                break;
            case 7:
                if(shootTimer.milliseconds() > 900) {
                    push.setPosition(kickUp);
                    intakeAndShoot.shoot();
                    shootTimer.reset();
                }
                if(actionTimer.getElapsedTimeSeconds() > 4.5) {
                    follower.followPath(secondLoad,true);
                    intakeAndShoot.setPos(0, intakePos);
                    intakeIndex = true;
                    intakeAndShoot.wallPos(0.25);
                    push.setPosition(kickZero);
                    setPathState(8);
                }

                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(acSecondLoad,.32,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.holdPoint(acIntakePose2);
                    actionTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10:
                if(actionTimer.getElapsedTimeSeconds() > .1) {
                    follower.followPath(scoreLoad2,true);
                    intakeAndShoot.setPos(0, intakePos);
                    intakeIndex = false;
                    intakeAndShoot.wallPos(0);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    actionTimer.resetTimer();
                    shootTimer.reset();
                    follower.holdPoint(scorePose1);
                    setPathState(12);
                }
                break;
            case 12:
                if(shootTimer.milliseconds() > 900) {
                    push.setPosition(kickUp);
                    intakeAndShoot.shoot();
                    shootTimer.reset();
                }
                if(actionTimer.getElapsedTimeSeconds() > 4.5) {
                    push.setPosition(kickZero);
                    intakeAndShoot.setPos(0, intakePos);
                    intakeAndShoot.wallPos(0.5);
                    follower.followPath(end,true);
                    setPathState(13);
                }
                break;

            case 13:
                if(!follower.isBusy()){
                    setPathState(14);
                }
                break;
            case 14:
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() { //this runs constantly during auto and we just update the position of the follower and check if it is still busy and cycle through each case
        hood.setPosition(.47);

        follower.update();
        intakeAndShoot.update(1, pathState, intakeIndex); //updating our shooter power and intake power
        try {
            autonomousPathUpdate(); //updating our cases so that we can change paths
        } catch (InterruptedException e) {
            throw new RuntimeException(e);

        }


        //telemetry debugs and current position tracking
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        //initializin all the different timers that we are going to use
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        //main thing that controls a lot
        intakeAndShoot = new intakeShoot(hardwareMap,"intake", "intake1",
                "shoot1", "shoot2",
                "spindexRoter", "slave",
                "disDiss", "dis2", "dis3",
                "dis4", "dis5", "dis6", "dis7", "wally");

        //thing that controls the servo that goes up and down allowing balls to shoot
        push = hardwareMap.get(Servo.class, "push");
        hood = hardwareMap.get(Servo.class, "shooterHood");
        //stuff that rotates the turret
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);


        //setting the basic positions of the hood and stuff
        hood.setPosition(hoodPos);
        turretLeft.setPosition(turTurn);
        turretRight.setPosition(turTurn);
        intakeAndShoot.setPos(0,1);


        buildPaths();
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