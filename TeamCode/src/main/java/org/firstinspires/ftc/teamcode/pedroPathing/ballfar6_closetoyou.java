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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AxonRotator;
import org.firstinspires.ftc.teamcode.ServoRotate;
import org.firstinspires.ftc.teamcode.colorShootFunc;

import java.util.List;

@Autonomous(name = "ballfar6_closetoyou", group = "Autonomous")
public class ballfar6_closetoyou extends OpMode {
    private Limelight3A limelight3A;
    AxonRotator smart;
    colorShootFunc hehe;
    private Follower follower; //this guy just kinda executes the paths type stuff yk
    private Timer pathTimer, actionTimer, opmodeTimer; //Path timer can be used in the autonomousPathUpdate just to see if one of the paths failed or something
    private int pathState; //just an int used later in autonomousPathUpdate for each of the cases (tells which path to do)
    private final Pose startPose = new Pose(8,-53, Math.toRadians(90));
    private final Pose intakePose1 = new Pose(8, -10, Math.toRadians(90));//this is where we should intake the BALLS idk where it is at this time so change late
    private final Pose acIntakePose1 = new Pose(8, -20 , Math.toRadians(90));
    private final Pose endPose1 = new Pose(15, -25, Math.toRadians(0));

    int index = 0;
    private Path score1;
    int scoonTrack = 0;
    int i = 0;

    int[] pattern = new int[] {2,1,2};

    boolean aiming = false;
    private PathChain firstLoad, secondLoad, acFirstLoad, acSecondLoad, end, scoreLoad1, scoreLoad2;
    private float timeSave=System.currentTimeMillis();
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
    double hoodPos = .12;
    double distance;
    double shooterPower;
    boolean index12 = true;
    boolean index1234 = false;
    CRServo spindexRoter;
    ServoRotate smart1;
    double offset = 60/360 * .4;
    public static double TURN_Constant = 0.01;
    boolean index123 = true;
    double turTurn = 0.05;
    boolean reverse = true;
    int Nonetwo = 120;
    int stageProg = 2;
    int[] ppg = new int[] {2,2,1};
    int[] gpp = new int[] {1,2,2};
    int[] pgp = new int[] {2,1,2};
    double limelightPause = System.currentTimeMillis();
    int indexForSpindex =0;
    boolean index1;
    boolean index12345 = true;
    boolean index123456 = false;
    ElapsedTime timer123=new ElapsedTime();
    ElapsedTime timer12=new ElapsedTime();
    ElapsedTime timer12345=new ElapsedTime();
    int actual = 120;
    DistanceSensor dis;
    double doos;
    Servo slave;
    private NormalizedColorSensor coloora;
    int[] actPattern = new int[3];

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
    public void buildPaths() {//this is where we build the path stuff
        score1 = new Path(new BezierLine(startPose, intakePose1));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), intakePose1.getHeading());
        firstLoad=follower.pathBuilder()
                .addPath(new BezierLine(intakePose1, acIntakePose1))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), acIntakePose1.getHeading())
                .build();
        acFirstLoad=follower.pathBuilder()
                .addPath(new BezierLine(acIntakePose1, startPose))
                .setLinearHeadingInterpolation(acIntakePose1.getHeading(), startPose.getHeading())
                .build();
        scoreLoad1= follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose1.getHeading())
                .build();

    }
    public void autonomousPathUpdate() throws InterruptedException {//we can add a lot more paths
        switch (pathState) {
            case 0:

                if(!follower.isBusy()){
                    Integralsum=0;
                    lasterror=0;
                    actionTimer.resetTimer();
                    follower.holdPoint(startPose);

                    setPathState(1);}
                break;
            case 1:
                if( actionTimer.getElapsedTimeSeconds() > 3 && !index1234){
                    index1234 = true;
                    index123 = false;
                }
                if(scoonTrack == 3 && !index123){

                    stageProg = hehe.score(pattern, stageProg);

                }
                if(actionTimer.getElapsedTimeSeconds() > 10) {
                    wally.setPosition(.5);
                    follower.followPath(score1,true);
                    stageProg = 2;
                    setPathState(2);
                }
                break;
            case 2:
                if(actionTimer.getElapsedTimeSeconds() > 15) {
                    wally.setPosition(.5);
                    follower.followPath(firstLoad,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(actionTimer.getElapsedTimeSeconds() > 15) {
                    wally.setPosition(.5);
                    follower.followPath(acFirstLoad,true);
                    hehe.reset();
                    setPathState(4);
                }
                break;
            case 4:

                if(!follower.isBusy()){
                    Integralsum=0;
                    lasterror=0;
                    actionTimer.resetTimer();
                    follower.holdPoint(startPose);

                    setPathState(5);}
                break;
            case 5:
                if( actionTimer.getElapsedTimeSeconds() > 3 && !index123456){
                    index123456 = true;
                    index12345 = false;
                }
                if(scoonTrack == 3 && !index123){

                    stageProg = hehe.score(pattern, stageProg);

                }
                if(actionTimer.getElapsedTimeSeconds() > 10) {
                    wally.setPosition(.5);
                    follower.followPath(scoreLoad1,true);
                    stageProg = 2;
                    setPathState(6);
                }
                break;
            case 6:
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
        telemetry.addData("disguisedjlsd ", doos);
        /*if (scoonTrack < 3) {
            scoonTrack = hehe.scOOON();
        }*/
        if (index == 0) {
            limelightPause = System.currentTimeMillis();
            index = 67; // I am an adult
        }
        follower.update();

        //double current = shooter2.getVelocity();
        //TargetVelocity = 1300;
        hood.setPosition(.52);
        //shooterPower = PIDControl(TargetVelocity+150, current);
        if (pathState != 12){
            scoonTrack = hehe.update(distance, 1, doos, Integralsum, lasterror, pathState, telemetry, 1);
            //scoonTrack = 3;
        }
        else {
            //scoonTrack = hehe.update(distance, 0, doos, Integralsum, lasterror, pathState, telemetry);
        }
        LLResult result = limelight3A.getLatestResult();
        if(result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
            for(LLResultTypes.FiducialResult tag: results){
                if (tag.getFiducialId() == 23) {
                    actPattern = ppg;
                } else if (tag.getFiducialId() == 22) {
                    actPattern = pgp;
                } else if (tag.getFiducialId() == 21) {
                    actPattern = gpp;
                }
                else if(tag.getFiducialId() == 24){
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
        timer12345.reset();
        coloora = hardwareMap.get(NormalizedColorSensor.class, "coloora");
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
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        slave = hardwareMap.get(Servo.class, "slave");
        // smart = hardwareMap.get(AxonRotator.class, "spindexRoter");
        smart1 = new ServoRotate(hardwareMap,"spindexRoter", "slave");
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
        timer12345.reset();
        //spindexRoter.setPower(0);
        hood.setPosition(hoodPos);
        hehe = new colorShootFunc(hardwareMap, smart1, shooter, shooter2, coloora, intake, intake1, wally, push);
        hehe.servRo.startRotate(hehe.servRo.getPosition(), 0, 360);
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
    public void stop() {}
}