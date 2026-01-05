package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp
public class TeleOp6221 extends OpMode {
    private Follower follower; //this guy just kinda executes the paths type stuff yk
    private Timer pathTimer, actionTimer, opmodeTimer; //Path timer can be used in the autonomousPathUpdate just to see if one of the paths failed or something
    private int pathState; //just an int used later in autonomousPathUpdate for each of the cases (tells which path to do)
    private Pose startPose = new Pose(65, -48, Math.toRadians(0)); // Start Pose of our robot. (I think these are the right measurements, as 0 degrees corresponds to facing right the starting x is a bit weird as it depends on where on the line we start)
    private final Pose scorePose1 = new Pose(97, -53, Math.toRadians(47)); // Scoring Pose of our robot. (Random for right now idk where we will score)
    private Limelight3A limelight3A;
    //DcMotor leftMotor;
    //DcMotor rightMotor;
    //DcMotor rightRearMotor;
    //DcMotor leftRearMotor;
    Drivetrain drivetrain;
    Servo turretRight; //launchservo
    Servo turretLeft; //launchservo
    CRServo artifactSpinner;
    Servo artifactPush;
    DcMotor shoot1;
    DcMotor shoot2;
    DcMotor intakeMotor;
    DcMotor intakeMotor1;
    Servo intakeGate;
    Servo shooterHood;

    double distance;
    double shooterPower;

    double kickZero = 0.85;
    double kickUp = 0.75;
    public boolean buttonPress;
    public boolean buttonCurrentlyPressed;
    public boolean pathFollow;
    private PathChain score1;

    public static double TURN_Constant = 0.005;
    public boolean aiming;
    double servoPos;
    public void buildPaths() {//this is where we build the path stuff
        startPose = new Pose(follower.getPose().getX(),follower.getPose().getY(),follower.getPose().getHeading() );
        score1=follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .setTimeoutConstraint(1500)
                .build();

    }
    public void autonomousPathUpdate() throws InterruptedException {//we can add a lot more paths
                if (!follower.isBusy() && buttonPress) {
                    follower.followPath(score1);
                    buttonPress = false;

                }
    }
    double lastTurretSpin = System.currentTimeMillis();

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);


        //drive
        //leftMotor = hardwareMap.get(DcMotor.class, "lr");
        //rightMotor = hardwareMap.get(DcMotor.class, "rr");
        //leftRearMotor = hardwareMap.get(DcMotor.class, "lf");
        //rightRearMotor = hardwareMap.get(DcMotor.class, "rf");
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);

        //intake
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");
        intakeGate = hardwareMap.get(Servo.class, "intakeGate");

        //artifact storage
        artifactSpinner = hardwareMap.get(CRServo.class, "spindexRoter");
        artifactPush = hardwareMap.get(Servo.class, "push");

        //shooter
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        shoot1 = hardwareMap.get(DcMotor.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotor.class, "shoot2");

        shooterHood = hardwareMap.get(Servo.class, "shooterHood");
        follower = Constants.createFollower(hardwareMap);
        aiming = true;
        shooterPower = 1;
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start(){
        limelight3A.start();
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
        //drive
        follower.update();
        if (!pathFollow){
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;
            drivetrain.arcadeDrive(-drive, strafe, twist);
        }
        LLResult result = limelight3A.getLatestResult();

        if(aiming){
        if (result != null && result.isValid()) {
            double ta = result.getTa();
            double tx = result.getTx();   // horizontal offset
            distance = 183.0242 * Math.pow(ta, -0.5161738);
            if(distance < 80) { //32 in to =cm
                shooterPower = 0.5;
            }
            else if(distance < 170){ //64 in to cm
                shooterPower = 0.75;
            }
            else if(distance < 200){
                shooterPower = 0.85;
            }
            else{
                shooterPower = 1;
            }
            double currentservo=turretLeft.getPosition();
            double turnCmd = TURN_Constant;
                if (tx<0){
                    turnCmd=-turnCmd;
                }
                double turn =currentservo+turnCmd;
                if(turn<0.005){
                    turn=0.994;
                }
                if(turn>0.995){
                    turn=0.006;
                }

                if (tx > 10 || tx < -10) {
                    turretLeft.setPosition(turn);
                    turretRight.setPosition(turn);
                }

                telemetry.addData("tx",tx);
                telemetry.addData("TurnCmd",turnCmd);
                telemetry.addData("currentservo",currentservo);
                telemetry.addData("turn",turn);
                //telemetry.update();

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        else{
            shooterPower = 1;
        }
        }

        //intake
        if(gamepad2.left_trigger > .5){ //active intake
            //intakeGate.setPower(0.75);
            intakeMotor.setPower(1);
            intakeMotor1.setPower(-1);
            //artifactSpinner.setPower(1);
        }
        else{
            intakeMotor1.setPower(0);
            intakeMotor.setPower(0);
        }

        if(gamepad2.a){
            intakeMotor1.setPower(1);
            intakeMotor.setPower(-1);
        }

        //artifact storage
        if(gamepad2.right_bumper){
            artifactSpinner.setPower(1);
            //artifactPush.setPosition(.85);
        }
        if(gamepad2.left_bumper){
            artifactSpinner.setPower(-1);
           // artifactPush.setPosition(.75);
        }

        if(gamepad2.y){
            artifactPush.setPosition(kickUp);
        }
        if(gamepad2.x){
            artifactPush.setPosition(kickZero);
        }
        if(gamepad2.dpad_down){
            if(!buttonCurrentlyPressed){
                if(!pathFollow){
                    buildPaths();
                    buttonPress = !buttonPress;
                    pathFollow=true;
                    follower.setStartingPose(startPose);
                }
            }
            buttonCurrentlyPressed = true;

            try {
                autonomousPathUpdate();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        else{
            buttonCurrentlyPressed = false;
        }
        if(pathFollow && !follower.isBusy()){
            pathFollow=false;

        }

        //shooter

        if(gamepad2.right_trigger > .5){
            //artifactPush.setPosition(-0.25);
           // artifactSpinner.setPower(-1);
            shoot1.setPower(-shooterPower);
            shoot2.setPower(shooterPower);
            //this should be autoaiming once it's done
        }
        else{
            //artifactPush.setPosition(-0.25);
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!gamepad2.right_bumper && (gamepad2.right_trigger < .5) && (gamepad2.left_trigger < .5) && !gamepad2.left_bumper && !gamepad2.a){
            artifactSpinner.setPower(0);
        }

        if(lastTurretSpin + 100 < System.currentTimeMillis()){
            servoPos = turretLeft.getPosition();
            lastTurretSpin = System.currentTimeMillis();
            if(gamepad2.dpad_left){
                aiming = false;
                if(turretLeft.getPosition() > 0.05){
                    turretLeft.setPosition(servoPos - TURN_Constant);
                    turretRight.setPosition(servoPos - TURN_Constant);
                }

            }
            else if(gamepad2.dpad_right){
                aiming = false;
                if(turretRight.getPosition() < 0.9){
                    turretLeft.setPosition(servoPos + TURN_Constant);
                    turretRight.setPosition(servoPos + TURN_Constant);
                }
            }
            else{
                aiming = true;
            }
        }

        telemetry.addData("left", turretLeft.getPosition());
        telemetry.addData("right", turretRight.getPosition());
        telemetry.addData("aiming",aiming);
        telemetry.addData("distance",distance);
        telemetry.addData("shooterPower",shooterPower);
        telemetry.update();
    }

}
