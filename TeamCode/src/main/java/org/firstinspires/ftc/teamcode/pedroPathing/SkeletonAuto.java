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

import org.firstinspires.ftc.teamcode.AutoTurret;
import org.firstinspires.ftc.teamcode.LimeLight;
import org.firstinspires.ftc.teamcode.intakeShoot;

@Autonomous(name = "SkeletonAuto", group = "Autonomous")
public class SkeletonAuto extends OpMode {
    private Follower follower; //this guy just kinda executes the paths type stuff yk
    private Timer pathTimer, actionTimer, opmodeTimer; //Path timer can be used in the autonomousPathUpdate just to see if one of the paths failed or something

    //positions
    private int pathState; //just an int used later in autonomousPathUpdate for each of the cases (tells which path to do)
    private final Pose startPose = new Pose(119.68,-19.9, -2.254); // Start Pose of our robot. (I think these are the right measurements, as 0 degrees corresponds to facing right the starting x is a bit weird as it depends on where on the line we start)

   private final Pose shootPose = new Pose(89.64,-54.57,-2.318);
   private final Pose tape3 = new Pose(35.53,-22.82,1.56);

    //paths
    private Path score1;
    private PathChain load1;

    private PathChain score2;
    private double IntegralSum = 0;
    private double lastError = 0;
    public static double Kp=0.0121;
    public static double Ki=0.00014;
    public static double Kd=0.0000;
    public static double Kf=.0000;
    //double savePosition = 0.0;

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
    boolean shoot = false;
    //boolean intake = false;
    //boolean shoot = false;
    AutoTurret turret;
    //ints
    //int index = 0;
    //int shootPos = 1;
    int intakePos = 0;
    LimeLight Lime;
    //int[] pattern = {1,2,2};
    //int[] ppg = {2,2,1};
    //int[] pgp = {2,1,2};
    //int[] gpp = {1,2,2};
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
        score1 = new Path(new BezierLine(startPose, shootPose));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        load1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, tape3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), tape3.getHeading(), .4)
                //  .setHeadingConstraint(.95)
                //  .setTranslationalConstraint(.95)
                .build();
        
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(tape3,shootPose))
                .setLinearHeadingInterpolation(tape3.getHeading(),shootPose.getHeading())
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
                actionTimer.resetTimer();
                //go to the next case
                setPathState(1);

                break;
            case 1:
                if(!follower.isBusy()&& shoot == false){
                    shoot = true;
                   actionTimer.resetTimer();
                    isShoot=true;
                }
                if(actionTimer.getElapsedTimeSeconds() > 5){
                    follower.followPath(load1);
                    actionTimer.resetTimer();
                    shoot = false;
                    isShoot = false;
                    intakeAndShoot.setPos(0,intakePos);
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()&& shoot == false) {
                    actionTimer.resetTimer();
                    shoot = true;
                }
                if (actionTimer.getElapsedTimeSeconds() > 5) {
                    intakeAndShoot.setPos(0,intakePos);
                    actionTimer.resetTimer();
                    follower.followPath(score2);
                    shoot = false;
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()&& shoot == false){
                    shoot = true;
                    actionTimer.resetTimer();
                    isShoot=true;
                }
                if(actionTimer.getElapsedTimeSeconds() > 6){
                    actionTimer.resetTimer();
                    shoot = false;
                    isShoot = false;
                    intakeAndShoot.setPos(0,intakePos);
                    setPathState(4);
                }
                break;

            case 4:
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
        turretLeft.setPosition(.18);
        turretRight.setPosition(.18);
        follower.update();
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
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);
        intakeAndShoot = new intakeShoot(hardwareMap,"intake", "intake1",
                "shoot1", "shoot2",
                "spindexRoter", "slave"
                , "wally", "color1", "color2", "shooterHood", follower);
        intakeAndShoot.setModeBlue();
        hood = hardwareMap.get(Servo.class, "shooterHood");
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turret = new AutoTurret(hardwareMap, "turretLeft", "turretRight");
        turret.setModeBlue();
        Lime = new LimeLight(hardwareMap);
        turretLeft.setPosition(.18);
        turretRight.setPosition(.18);
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