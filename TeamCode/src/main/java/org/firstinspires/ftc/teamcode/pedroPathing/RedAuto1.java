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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "RedAuto1", group = "Autonomous")
public class RedAuto1 extends OpMode {
    private Limelight3A limelight3A;
    private Follower follower; //this guy just kinda executes the paths type stuff yk
    private Timer pathTimer, actionTimer, opmodeTimer; //Path timer can be used in the autonomousPathUpdate just to see if one of the paths failed or something
    private int pathState; //just an int used later in autonomousPathUpdate for each of the cases (tells which path to do)
    private final Pose startPose = new Pose(8, -96, Math.toRadians(0)); // Start Pose of our robot. (I think these are the right measurements, as 0 degrees corresponds to facing right the starting x is a bit weird as it depends on where on the line we start)
    private final Pose scorePose1 = new Pose(90, -92, Math.toRadians(-49)); // Scoring Pose of our robot. (Random for right now idk where we will score)
    private final Pose intakePose1 = new Pose(81, -96, Math.toRadians(-90));//this is where we should intake the BALLS idk where it is at this time so change late
    private final Pose acIntakePose1 = new Pose(81, -134 , Math.toRadians(-90));
    private final Pose intakePose2 = new Pose(55, -98, Math.toRadians(-90));
    private final Pose acIntakePose2 = new Pose(55, -137, Math.toRadians(-90));
    private final Pose endPose1 = new Pose(65, -97, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(92, -92, Math.toRadians(-50));
    double kickZero = 0.85;
    double kickUp = 0.75;
    int index = 0;
    private Path score1;

    boolean aiming = false;
    private PathChain firstLoad, secondLoad, acFirstLoad, acSecondLoad, end, scoreLoad1, scoreLoad2, pause;
    private float timeSave=System.currentTimeMillis();
    double stay1;
    double stay2;
    DcMotorEx launcher;
    double Integralsum=0;
    public static double Kp=0.0047;
    public static double Ki=0.0001;
    public static double Kd=0;
    public static double Kf=0.0006;
    double TargetVelocity = 1200;
    ElapsedTime timer=new ElapsedTime();
    //ElapsedTime pidTimer = new ElapsedTime();
    double lasterror=0;
    DcMotorEx shooter, intake, shooter2, intake1;
    double turretPos = .45;
    double hoodPos = .25;
    double distance;
    double shooterPower;
    CRServo spindexRoter;
    public static double TURN_Constant = 0.01;

    double limelightPause = System.currentTimeMillis();

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
        return -114.21784*(dis*dis*dis)+518.54898*(dis*dis)-(855.8114*dis)+1801.48026;
    }
    public double getGoodHood(double dis){
        return- 0.145141*(dis*dis)+0.397149 * (dis) +0.30438;
    }
    //10/17 - got auto working but want to add functionality where the robot corrects to stay in the same place when paused
    public void buildPaths() {//this is where we build the path stuff
        score1 = new Path(new BezierLine(startPose, scorePose1));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());
        firstLoad=follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, intakePose1))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intakePose1.getHeading())
                .build();
        acFirstLoad=follower.pathBuilder()
                .addPath(new BezierLine(intakePose1, acIntakePose1))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), acIntakePose1.getHeading())
                .setTimeoutConstraint(1000)
                .build();
        scoreLoad1= follower.pathBuilder()
                .addPath(new BezierLine(acIntakePose1, scorePose1))
                .setLinearHeadingInterpolation(acIntakePose1.getHeading(), scorePose1.getHeading())
                .setTimeoutConstraint(100)
                .build();
        secondLoad= follower.pathBuilder()
                .addPath(new BezierLine(scorePose1,intakePose2))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intakePose2.getHeading())
                .setTimeoutConstraint(100)
                .build();
        acSecondLoad= follower.pathBuilder()
                .addPath(new BezierLine(intakePose2,acIntakePose2))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), acIntakePose2.getHeading())
                .setTimeoutConstraint(1000)
                .build();
        scoreLoad2= follower.pathBuilder()
                .addPath(new BezierLine(acIntakePose2, scorePose1))
                .setLinearHeadingInterpolation(acIntakePose2.getHeading(), scorePose1.getHeading())
                .setTimeoutConstraint(100)
                .build();
        end= follower.pathBuilder()
                .addPath(new BezierLine(scorePose1,endPose1))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), endPose1.getHeading())
                .setTimeoutConstraint(100)
                .build();
        pause = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1,scorePose3))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), scorePose3.getHeading())
                .setTimeoutConstraint(2500)
                .build();

    }
    public void autonomousPathUpdate() throws InterruptedException {//we can add a lot more paths
        switch (pathState) {
            case 0:
                push.setPosition(kickZero);
                //turretLeft.setPosition(0);
                //turretRight.setPosition(0);
                intake.setPower(1);
                intake1.setPower(-1);
                follower.followPath(score1);
                spindexRoter.setPower(0);
                //spindexRoter.setPower(-.3);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    //turretLeft.setPosition(0);
                    //turretRight.setPosition(0);
                    follower.followPath(pause);
                    //push.setPosition(kickUp);
                    //spindexRoter.setPower(0);

                    setPathState(2);}
                break;
            case 2:
                if (follower.isBusy()){
                    if (shooter2.getVelocity()>(.95*(TargetVelocity))){
                        wally.setPosition(0);
                        spindexRoter.setPower(-.5);
                        push.setPosition(kickUp);
                    }
                    else {
                        spindexRoter.setPower(.03);
                        wally.setPosition(0.2);
                        push.setPosition(kickZero);
                    }
                }
                if(!follower.isBusy()) {
                    push.setPosition(kickZero);
                    //turretLeft.setPosition(0);
                    //turretRight.setPosition(0);
                    spindexRoter.setPower(-.85);
                    //spindexRoter.setPower(.8);
                    //Thread.sleep(100);
                    follower.followPath(firstLoad,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    //spindexRoter.setPower(-.85);
                    //turretLeft.setPosition(0);
                    //turretRight.setPosition(0);
                    follower.followPath(acFirstLoad,true);
                    //spindexRoter.setPower(0);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    spindexRoter.setPower(-.85);
                    //turretLeft.setPosition(0);
                    //turretRight.setPosition(0);
                    follower.followPath(scoreLoad1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    //turretLeft.setPosition(0);
                    //turretRight.setPosition(0);
                    follower.followPath(pause,true);
                    //Thread.sleep(200);
                    //push.setPosition(kickUp);
                    setPathState(6);
                }
                break;
            case 6:
                if (follower.isBusy()){
                    if (shooter2.getVelocity()>(.95*(TargetVelocity))){
                        spindexRoter.setPower(-.4);
                        wally.setPosition(0);
                        push.setPosition(kickUp);
                    }
                    else {
                        spindexRoter.setPower(.03);
                        wally.setPosition(0.2);
                        push.setPosition(kickZero);
                    }
                }
                if(!follower.isBusy()) {
                    push.setPosition(kickZero);
                   // intake.setPower(-1);
                    //intake1.setPower(1);
                    //turretLeft.setPosition(0);
                    //turretRight.setPosition(0);
                    spindexRoter.setPower(-.85);
                    //Thread.sleep(100);
                    follower.followPath(secondLoad,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    push.setPosition(kickZero);
                    spindexRoter.setPower(-.85);
                    //intake.setPower(1);
                    //intake1.setPower(-1);
                    //turretLeft.setPosition(0);
                    //turretRight.setPosition(0);
                    follower.followPath(acSecondLoad,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    spindexRoter.setPower(0);
                    //turretLeft.setPosition(0);
                    //turretRight.setPosition(0);
                    follower.followPath(scoreLoad2,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    //spindexRoter.setPower(-1);
                    follower.followPath(pause,true);
                    //Thread.sleep(250);
                    //push.setPosition(kickUp);
                    setPathState(10);
                }
                break;
            case 10:
                if (follower.isBusy()){
                    if (shooter2.getVelocity()>(.95*(TargetVelocity))){
                        spindexRoter.setPower(-.4);
                        wally.setPosition(0);
                        push.setPosition(kickUp);
                    }
                    else {
                        spindexRoter.setPower(.03);
                        wally.setPosition(0.2);
                        push.setPosition(kickZero);
                    }
                }
                if(!follower.isBusy()) {
                    spindexRoter.setPower(0);
                    intake.setPower(0);
                    intake1.setPower(0);
                    push.setPosition(kickZero);
                    shooter.setPower(0);
                    shooter2.setPower(0);
                    follower.followPath(end,true);
                    //setPathState(10);
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
        if (index == 0) {
            limelightPause = System.currentTimeMillis();
            index = 67; // I am an adult
        }
        turretRight.setPosition(turretPos);
        follower.update();
        //turretLeft.setPosition(0.65);
        turretRight.setPosition(turretPos);
        LLResult result = limelight3A.getLatestResult();
        double current = shooter2.getVelocity();
        TargetVelocity = getGoodVel(distance)+100.0;
        hood.setPosition(getGoodHood(distance)-.04);
        shooterPower = PIDControl(TargetVelocity, current);
        if (pathState != 10){
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
            if(result != null && result.isValid()){
                List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
                for(LLResultTypes.FiducialResult tag: results){
                    if(tag.getFiducialId() == 24){
                        telemetry.addData("here", tag.getFiducialId());
                        double tx = tag.getTargetXDegrees();
                        distance = result.getTa();
                        /*double turnCmd = TURN_Constant;
                        if (tx<0){
                            turnCmd = -TURN_Constant;
                        }
                        double turn =turretLeft.getPosition() + turnCmd;
                        /*if(turn<0.005){
                            turn=0.994;
                        }
                        if(turn>0.995){
                            turn=0.006;
                        }

                        if (tx > 10 || tx < -10) {
                            //turretLeft.setPosition(turn);
                            turretRight.setPosition(turn);
                        }*/
                    }}}//}
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        telemetry.addData("Targetvelocity",TargetVelocity );
        telemetry.addData("acVel", shooter2.getVelocity());
        telemetry.addData("roter power", spindexRoter.getPower() );
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
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
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
        //stay1 = turretLeft.getPosition();
        //turretRight.getPosition();
        spindexRoter = hardwareMap.get(CRServo.class, "spindexRoter");
        up = hardwareMap.get(Servo.class, "intakeGate");
        hood = hardwareMap.get(Servo.class, "shooterHood");
        wally = hardwareMap.get(Servo.class, "wally");
        wally.setPosition(0);
        //shooterPower = 1;
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        //turretLeft.setPosition(0.65);
        turretRight.setPosition(turretPos);
        spindexRoter.setPower(0);
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
    public void stop() {}
}