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

//@Autonomous(name = "ExodusAuto", group = "Autonomous")
public class ExodusAuto extends OpMode {
    private Follower follower; //this guy just kinda executes the paths type stuff yk
    private Timer pathTimer, actionTimer, opmodeTimer; //Path timer can be used in the autonomousPathUpdate just to see if one of the paths failed or something

    //positions
    private int pathState; //just an int used later in autonomousPathUpdate for each of the cases (tells which path to do)

    private final Pose startPose = new Pose(129.64,-33.197, 1.52); // Start Pose of our robot. (I think these are the right measurements, as 0 degrees corresponds to facing right the starting x is a bit weird as it depends on where on the line we start)
    private final Pose shootPose = new Pose(126.165, -63.5, 1.5761);

    //paths
    private Path score1;

    //doubles
    double hoodPos = .25;
    double turTurn = 0.867;
    double kickZero = 0.85;
    double kickUp = 0.68;
    double TargetVelocity = 1200;
    double shooterPower = 0;
    AutoTurret turret;
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
    //ints
    int index = 0;
    int shootPos = 1;
    int intakePos = 0;
    LimeLight Lime;
    int[] pattern = {1,2,2};
    int[] ppg = {2,2,1};
    int[] pgp = {2,1,2};
    int[] gpp = {1,2,2};


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


    }
    public void autonomousPathUpdate() throws InterruptedException {//we can add a lot more paths
        switch (pathState) {

            case 0:
                    follower.followPath(score1);
                    actionTimer.resetTimer();
                    setPathState(1);
                break;
            case 1:

                if(actionTimer.getElapsedTimeSeconds() >= 11 && gate){
                    go = true;
                    isShoot = true;
                    gate= false;

                }
                if(actionTimer.getElapsedTimeSeconds() > 15) {
                    isShoot = false;
                    setPathState(2);
                }
                break;
            case 2:

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
        turret.updateAuto(follower, telemetry, intakeAndShoot.turretAngle(), scan);
        // turretLeft.setPosition(1);
        //turretRight.setPosition(1);
        intakeAndShoot.update(false,false, isShoot, false, follower, telemetry, false, false);
        intakeAndShoot.intakesetPower(1);
//.96
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
        turret = new AutoTurret(hardwareMap, "turretLeft", "turretRight");
        turret.setModeBlue();
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

        //thing that controls the servo that goes up and down allowing balls to shoot
        // push = hardwareMap.get(Servo.class, "push");
        hood = hardwareMap.get(Servo.class, "shooterHood");
        //stuff that rotates the turret
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");

        Lime = new LimeLight(hardwareMap);
        hood.setPosition(.4);




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