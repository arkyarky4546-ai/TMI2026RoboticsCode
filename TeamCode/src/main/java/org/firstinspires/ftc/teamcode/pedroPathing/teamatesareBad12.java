package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoTurret;
import org.firstinspires.ftc.teamcode.LimeLight;
import org.firstinspires.ftc.teamcode.intakeShoot;

@Autonomous(name = "SafeChud12BLue11", group = "Autonomous")
public class teamatesareBad12 extends OpMode {
    private Follower follower; //this guy just kinda executes the paths type stuff yk
    private Timer pathTimer, actionTimer, opmodeTimer; //Path timer can be used in the autonomousPathUpdate just to see if one of the paths failed or something

    //positions
    private int pathState; //just an int used later in autonomousPathUpdate for each of the cases (tells which path to do)
    private final Pose startPose = new Pose(128.8,-24.5, -2.35720214); // Start Pose of our robot. (I think these are the right measurements, as 0 degrees corresponds to facing right the starting x is a bit weird as it depends on where on the line we start)
    private final Pose scorePose1 = new Pose(84.14, -53.727, 2.22152); // Scoring Pose of our robot. (Random for right now idk where we will score)
    private final Pose scorePose2 = new Pose(79.67, -47.87, 2.836465);
    private final Pose intakePose1 = new Pose(54, -44, Math.toRadians(90));//this is where we should intake the BALLS idk where it is at this time so change late
    private final Pose acIntakePose1 = new Pose(58.75, -20.1058 , 1.5549);
    private final Pose intakePose2 = new Pose(65, -47, Math.toRadians(90));
    private final Pose hitPose = new Pose(60.4, -11 , .9852);
    private final Pose backPose = new Pose(84, -24, Math.toRadians(90));
    private final Pose acIntakePose2 = new Pose(83.414, -18, 1.573238);
    private final Pose intakePose3 = new Pose(35, -47, Math.toRadians(90));
    private final Pose acIntakePose3 = new Pose(34, -20, 1.58);
    private final Pose endPose1 = new Pose(58.368539, -10.4, .73678);
    private final Pose curve1 = new Pose(72.28, -51.71, 2.299);
    private final Pose curve11 = new Pose(61.59, -41.17, 1.7936);
    private final Pose curve2 = new Pose(84.4, -47.09, 1.77111);
    private final Pose curve21 = new Pose(83.714, -38.8, 1.573238);
    private final Pose curve3 = new Pose(61, -55, 2.5);
    private final Pose curve31 = new Pose(45, -47, 2.17);
    private final Pose hit1 = new Pose(58.9, -24.2, 1.15);
    private final Pose hit = new Pose(70.6, -39.5, 2.034);
    private final Pose scorePoseEnd = new Pose(113, -54.6, 2.54);


    //paths
    private Path score1;
    private PathChain firstLoad, secondLoad, acFirstLoad, acSecondLoad, end, scoreLoad1, scoreLoad2, thirdLoad, acThirdLoad, scoreLoad3, backLoad1, hitLoad, scoreLoad15, hiLoad;

    //doubles
    double hoodPos = .25;
    double turTurn = 0.867;
    double kickZero = 0.85;
    double kickUp = 0.68;
    double TargetVelocity = 1200;
    double shooterPower = 0;
    // AutoTurret turret;
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
    private Boolean scan = true;
    boolean gate = true;
    boolean intake = false;
    boolean shoot = false;
    AutoTurret turret;
    //ints
    int index = 0;
    int shootPos = 1;
    int intakePos = 0;
    LimeLight Lime;
    int[] pattern = {1,2,2};
    int[] ppg = {2,2,1};
    int[] pgp = {2,1,2};
    int[] gpp = {1,2,2};
    boolean auto = false;


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
        score1 = new Path(new BezierLine(startPose, scorePose1));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());

        /*firstLoad=follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, intakePose1))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intakePose1.getHeading())
                .build();*/
        acFirstLoad=follower.pathBuilder()
                .addPath(new BezierCurve(scorePose1, curve1, curve11, acIntakePose1))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), acIntakePose1.getHeading())
                //.setHeadingConstraint(.95)
                //.setTranslationalConstraint(.95)
                .build();
        scoreLoad1= follower.pathBuilder()
                .addPath(new BezierLine(acIntakePose1, scorePose1))
                .setLinearHeadingInterpolation(acIntakePose1.getHeading(), scorePose1.getHeading())
                // .setHeadingConstraint(.95)
                // .setTranslationalConstraint(.95)
                .build();
    /*    secondLoad= follower.pathBuilder()
                .addPath(new BezierLine(scorePose1,intakePose2))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intakePose2.getHeading())
                .build();*/
        backLoad1= follower.pathBuilder()
                .addPath(new BezierLine(acIntakePose1, backPose))
                .setLinearHeadingInterpolation(acIntakePose1.getHeading(), backPose.getHeading())
                // .setHeadingConstraint(.95)
                // .setTranslationalConstraint(.95)
                .build();
        scoreLoad15= follower.pathBuilder()
                .addPath(new BezierCurve(hitPose,hit1, hit, scorePose1))
                .setLinearHeadingInterpolation(hitPose.getHeading(), scorePose1.getHeading())
                // .setHeadingConstraint(.95)
                // .setTranslationalConstraint(.95)
                .build();
        hitLoad= follower.pathBuilder()
                .addPath(new BezierCurve(scorePose1,hit, hit1, hitPose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), hitPose.getHeading())
                //  .setHeadingConstraint(.95)
                // .setTranslationalConstraint(.95)
                .build();
        acSecondLoad= follower.pathBuilder()
                .addPath(new BezierCurve(scorePose1, curve2, curve21, acIntakePose2))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), acIntakePose2.getHeading())
                //  .setHeadingConstraint(.95)
                //  .setTranslationalConstraint(.95)
                .build();
        scoreLoad2= follower.pathBuilder()
                .addPath(new BezierLine(acIntakePose2, scorePose1))
                .setLinearHeadingInterpolation(acIntakePose2.getHeading(), scorePose1.getHeading())
                //  .setHeadingConstraint(.95)
                //  .setTranslationalConstraint(.95)
                .build();
        scoreLoad3= follower.pathBuilder()
                .addPath(new BezierLine(acIntakePose3, scorePoseEnd))
                .setLinearHeadingInterpolation(acIntakePose3.getHeading(), scorePoseEnd.getHeading())
                //  .setHeadingConstraint(.95)
                //  .setTranslationalConstraint(.95)
                .build();
        /*thirdLoad= follower.pathBuilder()
                .addPath(new BezierLine(scorePose1,intakePose3))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intakePose3.getHeading())
                .build();*/
        acThirdLoad= follower.pathBuilder()
                .addPath(new BezierCurve(scorePose1, curve3, curve31, acIntakePose3))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), acIntakePose3.getHeading())
                //  .setHeadingConstraint(.95)
                //  .setTranslationalConstraint(.95)
                .build();
        hiLoad= follower.pathBuilder()
                .addPath(new BezierCurve(scorePose1,hit, hit1, hitPose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), hitPose.getHeading())
                // .setHeadingConstraint(.95)
                // .setTranslationalConstraint(.95)
                .build();
        end= follower.pathBuilder()
                .addPath(new BezierLine(scorePose1,endPose1))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), endPose1.getHeading())
                //  .setHeadingConstraint(.95)
                //  .setTranslationalConstraint(.95)
                .build();
    }
    public void autonomousPathUpdate() throws InterruptedException {//we can add a lot more paths
        switch (pathState) {

            case 0:
                //the follower is now on the score1 path
                follower.followPath(score1);
                intakeAndShoot.setPos(0,intakePos);
                intakeIndex = false;
                isShoot = false;
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
                    shootTimer.reset();
                    //push.setPosition(kickUp);

                    setPathState(2);}
                break;
            case 2:

                if(actionTimer.getElapsedTimeSeconds() > .8 && gate){
                    go = true;
                    isShoot = true;
                    gate= false;

                }
                if(shootTimer.milliseconds() > 300 && go) {
                    //push.setPosition(kickUp);
                    //shooting every 800 milliseconds
                    isShoot = true;
                    go = false;

                    //this is what I mean about the timer being used to delay stuff
                    shootTimer.reset();
                }
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    isShoot = false;

                    follower.followPath(acFirstLoad);
                    go = true;
                    intakeAndShoot.setPos(0,intakePos);
                    //push servo is down now
                    //push.setPosition(kickZero);
                    //closed wall position
                    intakeIndex = true;
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.holdPoint(acIntakePose1);
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:

                if(actionTimer.getElapsedTimeSeconds() > .05) {
                    intakeAndShoot.setPos(0,intakePos);
                    intakeIndex = false;
                    follower.followPath(scoreLoad1);

                    setPathState(5);
                }
                break;
            case 5:
                if(follower.isBusy()){
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy()){
                    if(shootTimer.milliseconds() > 1200 && go) {
                        isShoot = true;
                        // push.setPosition(kickUp);
                        //shooting every 800 milliseconds
                        go = false;

                        //this is what I mean about the timer being used to delay stuff
                        shootTimer.reset();

                    }
                    if(actionTimer.getElapsedTimeSeconds() > 2.2) {
                        isShoot = false;
                        follower.followPath(hiLoad);

                        //push servo is down now
                        // push.setPosition(kickZero);
                        //closed wall position
                        intakeAndShoot.setPos(0,intakePos);
                        go = true;
                        intakeIndex = true;
                        actionTimer.resetTimer();
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if(follower.isBusy()){
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1.2){
                    follower.followPath(scoreLoad15);
                    auto = true;
                    actionTimer.resetTimer();
                    intakeAndShoot.setPos(0, intakePos);
                    setPathState(7);
                }
                break;
            case 7:
                if(actionTimer.getElapsedTimeSeconds()>.35){
                    auto = false;
                }

                if(!follower.isBusy()){

                    actionTimer.resetTimer();
                    shootTimer.reset();
                    follower.holdPoint(scorePose1);
                    // push.setPosition(kickUp);
                    setPathState(8);
                }
                break;
            case 8:
                if(shootTimer.milliseconds() > 1200 && go) {
                    //push.setPosition(kickUp);
                    isShoot = true;
                    go = false;
                    shootTimer.reset();
                }
                if(actionTimer.getElapsedTimeSeconds() > 2.2) {
                    follower.followPath(hiLoad);
                    intakeAndShoot.setPos(0,intakePos);
                    isShoot = false;
                    go = true;
                    intakeIndex = true;
                    //push.setPosition(kickZero);
                    actionTimer.resetTimer();
                    setPathState(9);
                }

                break;
            case 9:
                /*if(scan){
                    scan = false;
                    turretLeft.setPosition();
                    turretRight.setPosition();
                }*/
               /* if(Lime.getPatternFromLimelight() == 0){
                    pattern = gpp;

                }
                else if(Lime.getPatternFromLimelight() == 1){
                    pattern = pgp;

                }
                else if(Lime.getPatternFromLimelight() == 2){
                    pattern = ppg;

                }*/
                if(follower.isBusy()){
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1.1){
                    intakeAndShoot.setPos(0, intakePos);
                    // intakeAndShoot.findGreen();
                    follower.followPath(scoreLoad15);
                    auto = true;
                    actionTimer.resetTimer();
                    //intakeAndShoot.setPos(0,0);
                    scan = true;
                    setPathState(10);
                }
                break;
            case 10:
                /*if(intakeAndShoot.findGreen() != 0.0){
                    intakeAndShoot.colorSort(intakeAndShoot.findGreen(), pattern);

                }*/
                if(actionTimer.getElapsedTimeSeconds()>.4){
                    auto = false;
                }
                if(!follower.isBusy()){

                    actionTimer.resetTimer();
                    shootTimer.reset();
                    follower.holdPoint(scorePose1);
                    //intakeAndShoot.setPos(0,0);
                    //push.setPosition(kickUp);
                    setPathState(11);
                }
                break;
            case 11:
                if(shootTimer.milliseconds() > 1200 & go) {
                    //push.setPosition(kickUp);
                    isShoot = true;
                    //go = false;
                    shootTimer.reset();
                }
                if(actionTimer.getElapsedTimeSeconds() > 2.2) {
                    follower.holdPoint(scorePoseEnd);
                    isShoot = false;
                    go = true;
                    intakeIndex = true;
                    //push.setPosition(kickZero);
                    actionTimer.resetTimer();
                    setPathState(21);
                }

                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.holdPoint(acIntakePose2);
                    actionTimer.resetTimer();
                    setPathState(13);
                }
                break;
            case 13:
                if(actionTimer.getElapsedTimeSeconds() > .05) {
                    //intakeAndShoot.findGreen();
                    follower.followPath(scoreLoad2);
                    intakeAndShoot.setPos(0, intakePos);
                    intakeIndex = false;
                    setPathState(14);
                }
                break;
            case 14:

               /* if(intakeAndShoot.findGreen() != 0.0){
                    intakeAndShoot.colorSort(intakeAndShoot.findGreen(), pattern);

                }*/
                if(follower.getPathCompletion()>.9){
                    actionTimer.resetTimer();
                    shootTimer.reset();
                    follower.holdPoint(scorePose1);
                    //push.setPosition(kickUp);
                    setPathState(15);
                }
                break;
            case 15:
                if(shootTimer.milliseconds() > 200 && go) {
                    //push.setPosition(kickUp);
                    go = false;
                    isShoot = true;

                    //shooting every 800 milliseconds
                    //this is what I mean about the timer being used to delay stuff
                    shootTimer.reset();
                }
                if(actionTimer.getElapsedTimeSeconds() > 1) {
                    go = true;
                    isShoot = false;
                    follower.followPath(acThirdLoad);
                    intakeAndShoot.setPos(0, intakePos);
                    //push servo is down now
                    //push.setPosition(kickZero);
                    //closed wall position
                    intakeIndex = true;
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    follower.holdPoint(acIntakePose3);
                    actionTimer.resetTimer();
                    setPathState(17);
                }
                break;
            case 17:

                if(actionTimer.getElapsedTimeSeconds() > .05) {
                    // intakeAndShoot.findGreen();
                    auto = true;
                    follower.followPath(scoreLoad3);
                    intakeAndShoot.setPos(0, intakePos);
                    intakeIndex = false;

                    setPathState(18);
                }
                break;
            case 18:
                /*if(intakeAndShoot.findGreen() != 0.0){
                    intakeAndShoot.colorSort(intakeAndShoot.findGreen(), pattern);

                }*/
                if(follower.getPathCompletion()>.9){
                    auto = false;
                    actionTimer.resetTimer();
                    shootTimer.reset();
                    follower.holdPoint(scorePoseEnd);
                    //push.setPosition(kickUp);
                    setPathState(19);
                }
                break;
            case 19:
                if(shootTimer.milliseconds() > 200 && go) {
                    //push.setPosition(kickUp);
                    go = false;
                    isShoot = true;
                    shootTimer.reset();
                }
                if(actionTimer.getElapsedTimeSeconds() > 1) {
                    //push.setPosition(kickZero);
                    follower.holdPoint(scorePoseEnd);
                    go = true;
                    isShoot = false;
                    intakeAndShoot.setPos(0,intakePos);
                    setPathState(21);
                }
                break;

            case 21:
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /*public double shooterPowerSet(){
        double distanceFromGoal = Math.pow((Math.pow((144-follower.getPose().getX()),2) + Math.pow((144 + follower.getPose().getY()),2)) , .5);
        return 0.0000145 * Math.pow(distanceFromGoal, 4) - 0.00584813 * Math.pow(distanceFromGoal, 3) + 0.834897 * Math.pow(distanceFromGoal, 2) - 45.38315 * Math.pow(distanceFromGoal, 1) + 1930.07059;
    }
    public double hoodPosSet(){
        double distanceFromGoal = Math.pow((Math.pow((144-follower.getPose().getX()),2) + Math.pow((144+ follower.getPose().getY()),2)) , .5);
        return  -Math.pow(10, -9) * 2.0571 * Math.pow(distanceFromGoal, 4) - Math.pow(10, -7)*8.57305 * Math.pow(distanceFromGoal, 3) + 0.000313995 * Math.pow(distanceFromGoal, 2) - 0.0237158 * Math.pow(distanceFromGoal, 1) + 0.88;
    }
    public double getRecoil(){
        double distanceFromGoal = Math.pow((Math.pow((144-follower.getPose().getX()),2) + Math.pow((144 + follower.getPose().getY()),2)) , .5);
        return  -Math.pow(10, -9) * 5.66719 * Math.pow(distanceFromGoal, 4) + 0.00000199279 * Math.pow(distanceFromGoal, 3) -0.00024284 * Math.pow(distanceFromGoal, 2) +0.0127555 * Math.pow(distanceFromGoal, 1) -0.1900;
    }*/
    @Override
    public void loop() { //this runs constantly during auto and we just update the position of the follower and check if it is still busy and cycle through each case

        follower.update();
        turretLeft.setPosition(1);
        turretRight.setPosition(1);
        // turret.updateAutoAutoBlue(follower,telemetry, intakeAndShoot.turretAngle(),true);
        // turret.updateAuto(follower, telemetry, intakeAndShoot.turretAngle(), scan);
        // turretLeft.setPosition(1);
        //turretRight.setPosition(1);
        intakeAndShoot.update1(false,false, false, isShoot, false, follower, telemetry, auto,false);
        if(!auto) {
            intakeAndShoot.intakesetPower(1);
        }

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
        //turret = new AutoTurret(hardwareMap, "turretLeft", "turretRight");
        //turret.setModeBlue();
        //initializin all the different timers that we are going to use
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);
        //main thing that controls a lot
        intakeAndShoot = new intakeShoot(hardwareMap,"intake", "intake1",
                "shoot1", "shoot2",
                "spindexRoter", "slave"
                , "wally", "color1", "color2", "shooterHood", follower);
        intakeAndShoot.setModeBlue();
        //thing that controls the servo that goes up and down allowing balls to shoot
        // push = hardwareMap.get(Servo.class, "push");
        hood = hardwareMap.get(Servo.class, "shooterHood");
        //stuff that rotates the turret
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turret = new AutoTurret(hardwareMap, "turretLeft", "turretRight");
        turret.setModeBlue();
        turretLeft.setPosition(.99);
        turretRight.setPosition(.99);
        Lime = new LimeLight(hardwareMap);




        //setting the basic positions of the hood and stuff
        //hood.setPosition(hoodPos);
        //turretLeft.setPosition(turTurn);
        //turretRight.setPosition(turTurn);
        //intakeAndShoot.setPos(0,0);


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
    public void stop() {
        intakeAndShoot.stopT();
    }
}