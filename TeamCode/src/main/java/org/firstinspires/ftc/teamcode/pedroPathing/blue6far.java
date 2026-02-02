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
@Autonomous(name = "blue6far", group = "Autonomous")
public class blue6far extends OpMode {
    private Follower follower; //this guy just kinda executes the paths type stuff yk
    private Timer pathTimer, actionTimer, opmodeTimer; //Path timer can be used in the autonomousPathUpdate just to see if one of the paths failed or something

    //positions
    private int pathState; //just an int used later in autonomousPathUpdate for each of the cases (tells which path to do)
    private final Pose startPose = new Pose(8,-48, Math.toRadians(90)); // Start Pose of our robot. (I think these are the right measurements, as 0 degrees corresponds to facing right the starting x is a bit weird as it depends on where on the line we start)
    private final Pose intakePose1 = new Pose(8, -12, Math.toRadians(90));//this is where we should intake the BALLS idk where it is at this time so change late
    private final Pose acIntakePose1 = new Pose(8, -9.7 , Math.toRadians(90));
    private final Pose endPose1 = new Pose(65, -48, Math.toRadians(0));

    //paths
    private Path score1;
    private PathChain firstLoad, secondLoad, acFirstLoad, acSecondLoad, end, scoreLoad1, scoreLoad2, thirdLoad, acThirdLoad, scoreLoad3, backLoad1, hitLoad;

    //doubles
    double hoodPos = .45;
    double turTurn = .05;
    double kickZero = 0.85;
    double kickUp = 0.68;
    double TargetVelocity = 1200;
    double shooterPower = 0;
    double recoil = 0;
    private double IntegralSum = 0;
    private double lastError = 0;
    public static double Kp=0.0121;
    public static double Ki=0.00014;
    public static double Kd=0.0000;
    public static double Kf=.0000;
    double savePosition = 0.0;

    //timer
    private ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime PIDtimer=new ElapsedTime();

    //custom class that controls shooting and intake
    private intakeShoot intakeAndShoot;

    //servos
    Servo push, turretRight, turretLeft, hood;

    //booleans
    boolean intakeIndex = true;
    boolean isShoot = false;
    boolean go = false;
    //ints
    int index = 0;
    int shootPos = 1;
    int intakePos = 0;

    public double PIDControl(double reference, double state){
        double error=reference-state;
        double dt = PIDtimer.seconds();
        IntegralSum+=error*dt;
        double derivative=(error-lastError)/dt;
        lastError=error;
        PIDtimer.reset();
        return (error*Kp)+(derivative*Kd)+(IntegralSum*Ki)+(reference*Kf);
    }
    public void buildPaths() {//this is where we build the path stuff using our positions
        firstLoad=follower.pathBuilder()
                .addPath(new BezierLine(startPose, intakePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), intakePose1.getHeading())
                .build();
        acFirstLoad=follower.pathBuilder()
                .addPath(new BezierLine(intakePose1, acIntakePose1))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), acIntakePose1.getHeading())
                .build();
        scoreLoad1= follower.pathBuilder()
                .addPath(new BezierLine(acIntakePose1, startPose))
                .setLinearHeadingInterpolation(acIntakePose1.getHeading(), startPose.getHeading())
                .build();
        end= follower.pathBuilder()
                .addPath(new BezierLine(startPose,endPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose1.getHeading())
                .build();

    }
    public void autonomousPathUpdate() throws InterruptedException {//we can add a lot more paths
        switch (pathState) {

            case 0:
                //the follower is now on the score1 path
                intakeAndShoot.setPos(0,intakePos);
                intakeIndex = false;
                intakeAndShoot.wallPos(0.1);
                //go to the next case
                setPathState(1);

                break;
            case 1:
                    actionTimer.resetTimer();
                    follower.holdPoint(startPose);
                    //open wall position
                    intakeAndShoot.wallPos(0.1);
                    shootTimer.reset();
                    push.setPosition(kickUp);

                    setPathState(2);
                break;
            case 2:
                if(!go){
                    savePosition = hood.getPosition();
                }
                if(actionTimer.getElapsedTimeSeconds() > 2.5){
                    go = true;
                    isShoot = true;

                }
                if(shootTimer.milliseconds() > 1000 && go) {
                    push.setPosition(kickUp);
                    //shooting every 800 milliseconds
                    intakeAndShoot.simpleShoot();

                    hood.setPosition( savePosition);
                    savePosition -=recoil;

                    //this is what I mean about the timer being used to delay stuff
                    shootTimer.reset();
                }
                if(actionTimer.getElapsedTimeSeconds() > 7) {
                    intakeAndShoot.setPos(0, intakePos);
                    isShoot = false;
                    follower.followPath(firstLoad,true);

                    //push servo is down now
                    push.setPosition(kickZero);
                    //closed wall position
                    intakeAndShoot.wallPos(.4);
                    intakeIndex = true;
                    go = false;
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {

                    follower.followPath(acFirstLoad,.40,true);
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

                if(actionTimer.getElapsedTimeSeconds() > 3) {
                    intakeAndShoot.setPos(0,intakePos);
                    intakeAndShoot.wallPos(0.1);
                    intakeIndex = false;
                    follower.followPath(scoreLoad1,true);
                    IntegralSum = 0;
                    lastError = 0;
                    setPathState(6);
                }
                break;
            case 6:
                actionTimer.resetTimer();
                follower.holdPoint(startPose);
                //open wall position
                intakeAndShoot.wallPos(0.1);
                shootTimer.reset();
                push.setPosition(kickUp);
                go = false;

                setPathState(7);
                break;
            case 7:
                if(!go){
                    savePosition = hood.getPosition();
                }
                if(actionTimer.getElapsedTimeSeconds() > 2.0){
                    go = true;
                    isShoot = true;

                }
                if(shootTimer.milliseconds() > 1000 && go) {
                    push.setPosition(kickUp);
                    //shooting every 800 milliseconds
                    intakeAndShoot.simpleShoot();
                    isShoot = true;
                    hood.setPosition( savePosition);
                    savePosition -=recoil;

                    //this is what I mean about the timer being used to delay stuff
                    shootTimer.reset();
                }
                if(actionTimer.getElapsedTimeSeconds() > 5) {
                    intakeAndShoot.setPos(0, intakePos);
                    isShoot = false;
                    follower.followPath(firstLoad,true);

                    //push servo is down now
                    push.setPosition(kickZero);
                    //closed wall position
                    intakeAndShoot.wallPos(.4);
                    intakeIndex = true;
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {

                    follower.followPath(acFirstLoad,.40,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.holdPoint(acIntakePose1);
                    actionTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10:

                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    intakeAndShoot.setPos(0,intakePos);
                    intakeAndShoot.wallPos(0.1);
                    intakeIndex = false;
                    follower.followPath(scoreLoad1,true);
                    IntegralSum = 0;
                    lastError = 0;
                    if(opmodeTimer.getElapsedTimeSeconds() > 25){
                        setPathState(11);
                    }
                    else {
                        setPathState(6);
                    }
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(end);
                }

        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public double shooterPowerSet(){
        double distanceFromGoal = Math.pow((Math.pow((144-follower.getPose().getX()),2) + Math.pow((follower.getPose().getY()),2)) , .5);
        return 0.0000145 * Math.pow(distanceFromGoal, 4) - 0.00584813 * Math.pow(distanceFromGoal, 3) + 0.834897 * Math.pow(distanceFromGoal, 2) - 45.38315 * Math.pow(distanceFromGoal, 1) + 2300.07059;
    }
    public double hoodPosSet(){
        double distanceFromGoal = Math.pow((Math.pow((144-follower.getPose().getX()),2) + Math.pow((follower.getPose().getY()),2)) , .5);
        return  -Math.pow(10, -9) * 2.0571 * Math.pow(distanceFromGoal, 4) - Math.pow(10, -7)*8.57305 * Math.pow(distanceFromGoal, 3) + 0.000313995 * Math.pow(distanceFromGoal, 2) - 0.0237158 * Math.pow(distanceFromGoal, 1) + 0.87;
    }
    public double getRecoil(){
        double distanceFromGoal = Math.pow((Math.pow((144-follower.getPose().getX()),2) + Math.pow((follower.getPose().getY()),2)) , .5);
        return  -Math.pow(10, -9) * 5.66719 * Math.pow(distanceFromGoal, 4) + 0.00000199279 * Math.pow(distanceFromGoal, 3) -0.00024284 * Math.pow(distanceFromGoal, 2) +0.0127555 * Math.pow(distanceFromGoal, 1) -0.200;
    }
    @Override
    public void loop() { //this runs constantly during auto and we just update the position of the follower and check if it is still busy and cycle through each case

        follower.update();
        intakeAndShoot.update(1,1, intakeIndex);
        //intakeAndShoot.wallPos(.2);
        double current = Math.abs(intakeAndShoot.getVelocity());
        TargetVelocity = 1600;
        shooterPower = PIDControl(TargetVelocity, current);
        if(!isShoot) {

            hood.setPosition(.45);
        }
        recoil = .00;
        intakeAndShoot.shootsetPower(shooterPower);
        intakeAndShoot.intakesetPower(1);

        //intakeAndShoot.update(1, pathState, telemetry, intakeIndex); //updating our shooter power and intake power
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
        intakeAndShoot.setPos(0,0);


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