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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

//@TeleOp
public class PIDTUNE extends OpMode {
    public static double Kp=0.0047;
    public static double Ki=0.0004;
    public static double Kd=0;
    public static double Kf=0.0004;
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

    double distance;
    double shooterPower;

    double kickZero = 0.85;
    double kickUp = 0.75;
    double Integralsum=0;
    double hoodPos = .4;
    double turretPos = .48;
    public static double TURN_Constant = 0.005;

    double limelightPause = System.currentTimeMillis();
    private Limelight3A limelight3A;
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
        double current = shoot1.getVelocity();
        shooterPower = PIDControl(-TargetVelocity, current);
        if (index == 0) {
            limelightPause = System.currentTimeMillis();
            index = 67; // I am an adult
        }
        LLResult result = limelight3A.getLatestResult();
        if (limelightPause + 150 < System.currentTimeMillis()){
            limelightPause = System.currentTimeMillis();
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
                        }*/

                        /*if (tx > 10 || tx < -10) {
                            //turretLeft.setPosition(turn);
                            turretRight.setPosition(turn);
                        }*/
                    }}}}

        if (gamepad2.left_trigger > 0.5) {
            Integralsum=0;
            lasterror=0;
            shoot1.setPower(shooterPower);//shoot2 encoder not working
            shoot2.setPower(-shooterPower);
            if((shoot1.getVelocity()<(-TargetVelocity+100))&&(shoot1.getVelocity()>(-TargetVelocity-100))) {
                artifactPush.setPosition(kickUp);
                intakeMotor.setPower(1);
                intakeMotor1.setPower(-1);
            }else{
                artifactPush.setPosition(kickZero);
                intakeMotor.setPower(0);
                intakeMotor1.setPower(0);
            }
            artifactSpinner.setPower(-.8);
        } else if (gamepad2.right_trigger > 0.5) {
            Integralsum=0;
            lasterror=0;
            shoot1.setPower(shooterPower);
            shoot2.setPower(-shooterPower);
            intakeMotor.setPower(1);
            intakeMotor1.setPower(-1);
            if(sixSeven){
                artifactSpinner.setPower(0);
            }
            else{
                artifactSpinner.setPower(-.4);
            }
            if (!pressShooter) {
                pressShooter = true;
                timeBegan = System.currentTimeMillis();
            }
            if ((timeBegan + 150 < System.currentTimeMillis())) {
                if(shoot2.getVelocity()>(.92307692307*TargetVelocity)){
                    artifactPush.setPosition(kickUp);
                    sixSeven = false;
                }
                else {
                    artifactPush.setPosition(kickZero);
                    sixSeven = true;
                }
            }
            //turretLeft.setPosition();
            //turretRight.setPosition();
        } else {
            intakeMotor.setPower(0);
            intakeMotor1.setPower(0);
            artifactSpinner.setPower(0);
            shoot1.setPower(0);
            shoot2.setPower(0);
            pressShooter = false;
            artifactPush.setPosition(kickZero);
        }
        if(gamepad1.dpadUpWasPressed()){
            Kp+=.0001;
        }
        if(gamepad1.dpadDownWasPressed()){
            Kp-=.0001;
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
            Kf-=.000001;
        }
        if(gamepad1.bWasPressed()){
            Kf+=.00001;
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

        turretRight.setPosition(turretPos);
        turretLeft.setPosition(turretPos);
        shooterHood.setPosition(hoodPos);

        telemetry.addData("kp",Kp);
        telemetry.addData("kf",Kf);
        telemetry.addData("kd",Kd);
        telemetry.addData("ki",Ki);
        telemetry.addData("velocity1", shoot1.getVelocity());
        telemetry.addData("velocity2", shoot2.getVelocity());
        telemetry.addData("currentvelocity",current);
        telemetry.addData("shootPower", shooterPower);
        telemetry.addData("shooterHoodPos",hoodPos);
        telemetry.addData("turretPos",turretPos);
        telemetry.addData("distance", distance);
        telemetry.addData("target", TargetVelocity);

    }
    @Override
    public void init(){
        artifactSpinner = hardwareMap.get(CRServo.class, "spindexRoter");
        artifactPush = hardwareMap.get(Servo.class, "push");
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);
        //shooter
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterHood = hardwareMap.get(Servo.class, "shooterHood");
        turretRight.setPosition(turretPos);
        turretLeft.setPosition(turretPos);
        shooterHood.setPosition(hoodPos);
    }
    @Override
    public void start(){
        limelight3A.start();
        //opmodeTimer.resetTimer();
    }
}
