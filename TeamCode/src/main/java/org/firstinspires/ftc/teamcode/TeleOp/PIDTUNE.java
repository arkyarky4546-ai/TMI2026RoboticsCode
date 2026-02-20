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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intakeShoot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp
public class PIDTUNE extends OpMode {
    private double IntegralSum = 0;

    private Follower follower;
    private double lastError = 0;
    public static double Kp=0.0121;
    public static double Ki=0.00014;
    public static double Kd=0.0000;
    public static double Kf=.0000;
    double TargetVelocity = 1400;
    private double PIDdistance = 0;
    private double timeBegan;
    private boolean pressShooter;
    boolean sixSeven = false;
    ElapsedTime timer=new ElapsedTime();
    //ElapsedTime pidTimer = new ElapsedTime();
    double lasterror=0;
    ServoImplEx turretRight; //launchservo
    ServoImplEx turretLeft; //launchservo
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
    boolean isShooting = false;
    double lastPos = 0.0;
    double kickZero = 0.85;
    double kickUp = 0.65;
    double Integralsum=0;
    double hoodPos = .4;
    double turretPos = .96;
    double recoil = .03;
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
        return output;
    }

    @Override
    public void loop() {
        follower.update();
        //hoodPos();
        intakeAndShoot.wallPos(.2);
        double current = Math.abs(intakeAndShoot.getVelocity());

        shooterPower = PIDControl(TargetVelocity, current);
        intakeAndShoot.shootsetPower(shooterPower);
        intakeAndShoot.intakesetPower(1);
        intakeAndShoot.update(1,1, intakeIndex);
        if (gamepad2.left_trigger > 0.5) {
            if (!intakeIndex) {
                intakeIndex = true;
                intakeAndShoot.setPos(0, 0);
                isShooting = false;

            }
        }
        if (gamepad2.right_trigger > 0.5) {
            if(intakeIndex){
                intakeIndex = false;
                shootTimer1.reset();
                intakeAndShoot.setPos(0,0);
                isShooting = true;
            }

            telemetry.addData("hit", 0);
            if(shootTimer1.milliseconds() > 400){
                intakeAndShoot.wallPos(0);
                intakeAndShoot.simpleShoot();
                hood.setPosition(hood.getPosition()-recoil);
                shootTimer1.reset();
            }
        }
        else{
            //1intakeAndShoot.wallPos(.5);
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
            recoil -=.005;
        }
        if(gamepad2.dpadRightWasPressed()){
            recoil += .005;
        }
        if(gamepad2.leftBumperWasPressed()){
            push.setPosition(kickZero);
        }
        if(gamepad2.rightBumperWasPressed()){
            push.setPosition(kickUp);
        }
        if(!isShooting) {
            hood.setPosition(hoodPos);
        }
        if(gamepad1.dpadUpWasPressed()){
            Kp+=.001;
        }
        if(gamepad1.dpadDownWasPressed()){
            Kp-=.001;
        }
        if(gamepad1.dpadRightWasPressed()){
            Kd+=.0001;
        }
        if(gamepad1.dpadLeftWasPressed()){
            Kd-=.0001;
        }
        if(gamepad1.yWasPressed()){
            Ki+=.00001;
        }
        if(gamepad1.aWasPressed()){
            Ki-=.00001;
        }
        if(gamepad1.xWasPressed()){
            Kf-=.00005;
        }
        if(gamepad1.bWasPressed()){
            Kf+=.00005;
        }
        turretRight.setPosition(turretPos);
        turretLeft.setPosition(turretPos);
        telemetry.addData("velocity1", intakeAndShoot.getVelocity());
        telemetry.addData("hoodrecoil", recoil);
        // telemetry.addData("velocity2", );
        telemetry.addData("shootPower", shooterPower);
        telemetry.addData("shooterHoodPos",hoodPos);
        telemetry.addData("turretPos",turretPos);
        telemetry.addData("distance", distance);
        telemetry.addData("target", TargetVelocity);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("p", Kp);
        telemetry.addData("d", Kd);
        telemetry.addData("i", Ki);
        telemetry.addData("f", Kf);

    }
    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);

        follower.setPose(new Pose(128,-26, Math.toRadians(47)));
        follower.update();
        intakeAndShoot = new intakeShoot(hardwareMap,"intake", "intake1",
                "shoot1", "shoot2",
                "spindexRoter", "slave",
                "disDiss", "dis2", "dis3",
                "dis4", "dis5", "dis6", "dis7", "wally", "color1", "color2", "shooterHood");
        intakeAndShoot.wallPos(.2);
        hood = hardwareMap.get(Servo.class, "shooterHood");
        push = hardwareMap.get(Servo.class, "push");
        turretRight = hardwareMap.get(ServoImplEx.class, "turretRight");
        turretRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turretLeft = hardwareMap.get(ServoImplEx.class, "turretLeft");
        turretLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeAndShoot.setPos(0,0);

    }

    @Override
    public void start(){

        //opmodeTimer.resetTimer();
    }
}




