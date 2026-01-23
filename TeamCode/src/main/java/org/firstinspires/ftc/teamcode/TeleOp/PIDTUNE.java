package org.firstinspires.ftc.teamcode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intakeShoot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp
public class PIDTUNE extends OpMode {
    private double IntegralSum = 0;

    private Follower follower;
    private double lastError = 0;
    public static double Kp=0.0047;
    public static double Ki=0.0004;
    public static double Kd=0;
    public static double Kf=0;
    double TargetVelocity = 1400;
    private double timeBegan;
    private boolean pressShooter;
    boolean sixSeven = false;
    ElapsedTime timer=new ElapsedTime();
    //ElapsedTime pidTimer = new ElapsedTime();
    double lasterror=0;
    Servo turretRight; //launchservo
    Servo turretLeft; //launchservo
    CRServo artifactSpinner;
    Servo artifactPush;
    DcMotor intake;
    DcMotor intake1;
    DcMotorEx shoot1;
    DcMotorEx shoot2;
    DcMotor intakeMotor;
    DcMotor intakeMotor1;
    Servo intakeGate;
    Servo shooterHood;
    Servo push;
    Servo hood;
    intakeShoot intakeAndShoot;
    double distance;
    double shooterPower;
    boolean intakeIndex = true;
    double kickZero = 0.85;
    double kickUp = 0.65;
    double Integralsum=0;
    double hoodPos = .4;
    double turretPos = .48;
    public static double TURN_Constant = 0.005;
    private ElapsedTime shootTimer1 = new ElapsedTime();
    double limelightPause = System.currentTimeMillis();
    int index = 0;
    public double PIDControl(double reference, double state){
        double error=reference-state;
        double dt = timer.seconds();
        Integralsum+=error*dt;
        double derivative=(error-lasterror)/dt;
        lasterror=error;
        timer.reset();

        double output=(error*Kp)+(derivative*Kd)+(Integralsum*Ki)+(reference*Kf);
        return output/6;
    }
    @Override
    public void loop() {
        follower.update();

        double current = Math.abs(intakeAndShoot.getVelocity());
        shooterPower = PIDControl(TargetVelocity, current);
        intakeAndShoot.shootsetPower(shooterPower);
        intakeAndShoot.intakesetPower(1);
        intakeAndShoot.update(1,1,telemetry, intakeIndex);
        if (gamepad2.left_trigger > 0.5) {
            if (!intakeIndex) {
                intakeIndex = true;
                intakeAndShoot.setPos(0, 0);
            }
        }
        if (gamepad2.right_trigger > 0.5) {
            intakeAndShoot.wallPos(0);
            if(intakeIndex){
                intakeIndex = false;
                intakeAndShoot.setPos(0,0);
            }

            telemetry.addData("hit", 0);
            if(shootTimer1.milliseconds() > 500){

                intakeAndShoot.simpleShoot();
                shootTimer1.reset();
            }
        }
        else{
            intakeAndShoot.wallPos(.5);
        }
        if(gamepad2.aWasPressed()){
            hoodPos-=.01;
        }
        if(gamepad2.yWasPressed()){
            hoodPos+=.01;
        }
        if(gamepad2.xWasPressed()){
            turretPos-=.01;
        }
        if(gamepad2.bWasPressed()){
            turretPos+=.01;
        }
        if(gamepad2.dpadUpWasPressed()){
            TargetVelocity+=25;
        }
        if(gamepad2.dpadDownWasPressed()){
            TargetVelocity-=25;
        }
        if(gamepad2.dpadLeftWasPressed()){
            intakeAndShoot.setPos(0, 0);
        }
        if(gamepad2.dpadRightWasPressed()){
            intakeAndShoot.setPos(0, 1);
        }
        if(gamepad2.leftBumperWasPressed()){
            push.setPosition(kickZero);
        }
        if(gamepad2.rightBumperWasPressed()){
            push.setPosition(kickUp);
        }

        turretRight.setPosition(turretPos);
        turretLeft.setPosition(turretPos);
        hood.setPosition(hoodPos);

        telemetry.addData("velocity1", intakeAndShoot.getVelocity());
       // telemetry.addData("velocity2", );
        telemetry.addData("shootPower", shooterPower);
        telemetry.addData("shooterHoodPos",hoodPos);
        telemetry.addData("turretPos",turretPos);
        telemetry.addData("distance", distance);
        telemetry.addData("target", TargetVelocity);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

    }
    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        follower.update();
        intakeAndShoot = new intakeShoot(hardwareMap,"intake", "intake1",
                "shoot1", "shoot2",
                "spindexRoter", "slave",
                "disDiss", "dis2", "dis3",
                "dis4", "dis5", "dis6", "dis7", "wally");

        hood = hardwareMap.get(Servo.class, "shooterHood");
        push = hardwareMap.get(Servo.class, "push");
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        intakeAndShoot.setPos(0,0);

    }
    @Override
    public void start(){

        //opmodeTimer.resetTimer();
    }
}
