package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AxonRotator;

import java.util.List;

@Autonomous(name = "RedNear3", group = "Autonomous")
public class RedNear3 extends OpMode {
    private Limelight3A limelight3A;
    AxonRotator smart;
    private Follower follower; //this guy just kinda executes the paths type stuff yk
    private Timer pathTimer, actionTimer, opmodeTimer; //Path timer can be used in the autonomousPathUpdate just to see if one of the paths failed or something
    private int pathState; //just an int used later in autonomousPathUpdate for each of the cases (tells which path to do)
    private final Pose startPose = new Pose(8,-96, Math.toRadians(0)); // Start Pose of our robot. (I think these are the right measurements, as 0 degrees corresponds to facing right the starting x is a bit weird as it depends on where on the line we start)
    private final Pose endPose = new Pose(20, -96, Math.toRadians(0));
    double kickZero = 0.85;
    double kickUp = 0.75;
    int index = 0;


    boolean aiming = false;
    private PathChain forward, pause, backward;
    private float timeSave=System.currentTimeMillis();
    double stay1;
    double stay2;
    DcMotorEx launcher;
    double Integralsum=0;
    public static double Kp=0.0047;
    public static double Ki=0.0004;
    public static double Kd=0;
    public static double Kf=0;
    double TargetVelocity = 1300;
    ElapsedTime timer=new ElapsedTime();
    //ElapsedTime pidTimer = new ElapsedTime();
    double lasterror=0;
    DcMotorEx shooter, intake, shooter2, intake1;
    double turretPos = 1;
    double hoodPos = .25;
    double distance;
    double shooterPower;
    CRServo spindexRoter;
    AxonRotator smart1;
    public static double TURN_Constant = 0.01;
    double turTurn = .5;
    boolean reverse = true;
    int Nonetwo = 120;
    int Nsix = -60;
    double limelightPause = System.currentTimeMillis();
    int indexForSpindex =0;
    boolean index1;
    ElapsedTime timer123=new ElapsedTime();
    ElapsedTime timer12=new ElapsedTime();
    int actual = 120;
    DistanceSensor dis;
    double doos;
    CRServo slave;

    Servo push, turretRight, turretLeft, up, hood, wally;
    public double PIDControl(double reference, double state){
        double error=reference-state;
        double dt = timer.seconds();
        Integralsum+=error*dt;
        double derivative=(error-lasterror)/dt;
        lasterror=error;
        timer.reset();

        double output=(error*Kp)+(derivative*Kd)+(Integralsum*Ki)+(reference*Kf);
        return output;
    }
    public double getGoodVel(double dis){
        return -217*(dis*dis*dis) + 875.6403*(dis*dis) -1196.11498*(dis) + 1830.8098;
    }
    /* public double getGoodVel(double dis){
         return -114.21784*(dis*dis*dis)+518.54898*(dis*dis)-(855.8114*dis)+1801.48026;
     }
     public double getGoodHood(double dis){
         return- 0.145141*(dis*dis)+0.397149 * (dis) +0.30438;
     }*/

    public void buildPaths() {//this is where we build the path stuff
        forward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .setTimeoutConstraint(100)
                .build();
    }
    public void autonomousPathUpdate() throws InterruptedException {//we can add a lot more paths
        switch (pathState) {
            case 0:
                Integralsum=0;
                lasterror=0;
                push.setPosition(kickZero);
                intake.setPower(1);
                intake1.setPower(-1);
                //follower.followPath(backward);
                //spindexRoter.setPower(0);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    Integralsum=0;
                    lasterror=0;
                    actionTimer.resetTimer();
                    follower.holdPoint(startPose);
                    setPathState(2);}
                break;
            case 2:
                actual = 120;
                if (index1) {
                    actual = 60;
                }
                if(shooter2.getVelocity()>(.87*TargetVelocity)){

                    wally.setPosition(0);
                    if (indexForSpindex == 0){
                        index1 = false;
                        timer123.reset();
                        smart1.startRotate(actual);
                        //spoon+=1;
                        //smart2.startRotate120(actual);
                        indexForSpindex += 1;
                    }
                    else if (indexForSpindex ==2 && timer123.seconds() > 0.35){
                        //spoon+=1;
                        smart1.startRotate(actual);
                        //smart2.startRotate120(actual);
                        indexForSpindex = 0;
                    }
                    else if (indexForSpindex ==1 && timer123.seconds() > 0.2){
                        smart1.startRotate(actual);
                        //spoon+=1;
                        //smart2.startRotate120(actual);
                        indexForSpindex += 1;
                    }
                    push.setPosition(kickUp);

                    //gateShoot = false;
                }

                else {
                    //spindexRoter.setPower(.03);

                    wally.setPosition(0.2);
                    push.setPosition(kickZero);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1.5) {
                    //spindexRoter.setPower(0);
                    push.setPosition(kickZero);
                    indexForSpindex = 0;
                    follower.followPath(forward,true);
                    setPathState(3);
                }
                break;
            case 3:
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() { //this runs constantly during auto and we just update the position of the follower and check if it is still busy and cycle through each case
        doos = dis.getDistance(DistanceUnit.CM);
        smart1.update(telemetry);
        if (index == 0) {
            limelightPause = System.currentTimeMillis();
            index = 67; // I am an adult
        }
        //turretRight.setPosition(turTurn);
        // turretLeft.setPosition(turTurn);
        follower.update();

        double current = shooter2.getVelocity();
        TargetVelocity = 1300;
        hood.setPosition(.48);
        shooterPower = PIDControl(TargetVelocity+150, current);
        if (pathState != 12){
            shooter.setPower(-shooterPower);
            shooter2.setPower(shooterPower);
        }
        else {
            shooter.setPower(0);
            shooter2.setPower(0);
        }
        //telemetry.addData("shootperwer",shooterPower);
        //telemetry.addData("velocity",current);

        //if (limelightPause + 150 < System.currentTimeMillis()){
        //limelightPause = System.currentTimeMillis();
        LLResult result = limelight3A.getLatestResult();
        if(result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
            for(LLResultTypes.FiducialResult tag: results){
                if(tag.getFiducialId() == 24){
                    double toBeSetPos = turretLeft.getPosition();
                    double tx = tag.getTargetXDegrees();
                    if (tx < -7) {
                        toBeSetPos -= 0.01;
                    } else if (tx > 7) {
                        toBeSetPos += 0.01;
                    }
                    turretLeft.setPosition(toBeSetPos);
                    turretRight.setPosition(toBeSetPos);
                }}}
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        telemetry.addData("Targetvelocity",TargetVelocity );
        telemetry.addData("acVel", shooter2.getVelocity());
        // telemetry.addData("roter power", spindexRoter.getPower() );
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
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        dis = hardwareMap.get(DistanceSensor.class, "disDiss");
        limelight3A.pipelineSwitch(8);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        push = hardwareMap.get(Servo.class, "push");
        shooter = hardwareMap.get(DcMotorEx.class, "shoot1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        slave = hardwareMap.get(CRServo.class, "slave");
        // smart = hardwareMap.get(AxonRotator.class, "spindexRoter");
        smart1 = new AxonRotator(hardwareMap,"spindexRoter", "slave", "smartTrack", reverse);
        //stay1 = turretLeft.getPosition();
        //turretRight.getPosition();
        //spindexRoter = hardwareMap.get(CRServo.class, "spindexRoter");
        up = hardwareMap.get(Servo.class, "intakeGate");
        hood = hardwareMap.get(Servo.class, "shooterHood");
        wally = hardwareMap.get(Servo.class, "wally");
        wally.setPosition(0);
        //shooterPower = 1;
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        turretLeft.setPosition(turTurn);
        turretRight.setPosition(turTurn);
        //spindexRoter.setPower(0);
        hood.setPosition(hoodPos);

    }



    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        limelight3A.start();
        setPathState(0);
    }

    @Override
    public void stop() {}}