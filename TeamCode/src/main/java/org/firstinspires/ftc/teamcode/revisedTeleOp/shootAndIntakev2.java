package org.firstinspires.ftc.teamcode.revisedTeleOp;

import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.ShooterConstants;
import org.firstinspires.ftc.teamcode.servo720Rot;
import org.firstinspires.ftc.teamcode.shooterThread;

public class shootAndIntakev2 {
    //intake
    DcMotorEx intake1;
    DcMotorEx intake2;
    //Servo intakeGate;

    //both
    private final double kickZero = 0.85;
    private final double kickUp = 0.70;

    //servo720Rot
    servo720Rot servRot;

    //shooter
    Servo turretRight;
    Servo turretLeft;
    DcMotorEx shoot1;
    DcMotorEx shoot2;
    Servo wall;
    Servo shooterHood;
    private final double TURRET_START = 0.96;

    double Integralsum = 0;
    public static double Kp=0.0121;
    public static double Ki=0.00014;
    public static double Kd=0.0000;
    public static double Kf=.0000;
    private double currentVelocity;
    public double targetVelocity;

    public double recoil;
    private double intakeDis = 0.0;
    double power = 0.0;
    boolean index1 = false;
    boolean sorted = false;
    boolean isShoot = false;
    double lasterror = 0;
    ElapsedTime PIDTimer = new ElapsedTime();
    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime updateTimer = new ElapsedTime();
    ElapsedTime shootingTimer = new ElapsedTime();
    ElapsedTime intakeUpdate = new ElapsedTime();
    final double intakePower = 1.0; //TODO: find actual power
    double offset  = 400/360*2/5 * 360/355 * 20/18;
    double gearOff = 360/355 * 20/18;
    private regressCalc regression;
    //private sensCalc sensors;
    private boolean shooting = false;

    shooterThread shootThing;
    Shooter shooter;

    public shootAndIntakev2(HardwareMap hardwareMap, Follower follower){
        intake1 = hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake1");
        //intakeGate = hardwareMap.get(Servo.class, "intakeGate");
        //intakeGate.setPosition(0);

        servRot = new servo720Rot(hardwareMap,"spindexRoter", "slave", "color1", "color2");

        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turretRight.setPosition(TURRET_START);
        turretLeft.setPosition(TURRET_START);
        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        wall = hardwareMap.get(Servo.class, "wally");
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterHood = hardwareMap.get(Servo.class, "shooterHood");
        shooterHood.setPosition(0.4);

        regression = new regressCalc();
        //sensors = new sensCalc(servRot);
        PIDTimer.reset();
        shootTimer.reset();
        regression.start();
        //sensors.start();

        shooter = new Shooter();
        shootThing = new shooterThread(shooter, follower, ShooterConstants.GOAL_POSE_BLUE, follower.getHeading()); //make this work by doing stuff
        shootThing.start();

    }
    public void stopT(){
        //sensors.stopThread();
        regression.stopThread();
        shootThing.stopThread();
    }
    /*public void update(double distance, boolean intakeActive, boolean shootActive, boolean intakeOut,boolean servoReset, Telemetry telemetry, boolean shoot1300){
        if(servoReset){
            servRot.sSP(0,0);
            index1 = false;
        }
        if(updateTimer.milliseconds() > 30) {
            regression.setDis(distance);
            currentVelocity = abs(shoot1.getVelocity());
            targetVelocity = regression.vel();
            power = shooterPIDControl(targetVelocity, currentVelocity) / 2;
            shoot1.setPower(-power);
            shoot2.setPower(power);
            recoil = regression.rec();
            if (!isShoot) {
                shooterHood.setPosition(regression.hoo());
            }
            updateTimer.reset();
        }
        if(intakeActive){
            intakeDis = sensors.getIntakeDistance();
            shooting = false;
            isShoot = false;
            artifactPush.setPosition(kickZero);
            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            wall.setPosition(.5);
            if(intakeDis < 10 && intakeTimer.milliseconds() > 271 && !isShoot) {
                servRot.regRot(servRot.getPos());
                intakeTimer.reset();
            }

        }
        else if(intakeOut) {
            intake1.setPower(-intakePower);
            intake2.setPower(intakePower);
            artifactPush.setPosition(kickZero);
        }
        else if(shootActive){
            if (!shooting){
                shooting = true;
                isShoot = true;
                shootingTimer.reset();
                wall.setPosition(0);
            }
            if(shootingTimer.milliseconds() > 500 ) {
                artifactPush.setPosition(kickUp);

                intake1.setPower(1);
                intake2.setPower(-1);

                if (shootTimer.milliseconds() > 400) {
                    servRot.regRot(servRot.getPos());
                    shooterHood.setPosition(shooterHood.getPosition() - recoil);
                    shootTimer.reset();
                }
            }
            if(shoot1300){
                targetVelocity = 1300;
            }
            power = shooterPIDControl(targetVelocity, currentVelocity);
            shoot1.setPower(-power);
            shoot2.setPower(power);

        }
        else{
            intake1.setPower(0);
            isShoot = false;
            intake2.setPower(0);
            artifactPush.setPosition(kickZero);
            shooting = false;
            //servRot.sSP(0,0);
            shootTimer.reset();
        }

    } */

    //override update thingy for the blue for right now
    public void update(boolean intakeActive, boolean shootActive, boolean intakeOut,boolean servoReset, Telemetry telemetry, boolean shoot1300, boolean xPress, int[] pattern, Follower follower){
        shootThing.update(follower, ShooterConstants.GOAL_POSE_BLUE, follower.getHeading());
        if(servoReset){
            servRot.sSP(0,0);
            index1 = false;
        }
        if(updateTimer.milliseconds() > 30) {
            shoot1.setVelocity(-shootThing.getSpeed());
            shoot2.setVelocity(shootThing.getSpeed());
            shooterHood.setPosition(shootThing.getHoodPos());
            updateTimer.reset();
        }
        if(intakeActive){
            //intakeDis = sensors.getIntakeDistance();
            shooting = false;
            isShoot = false;
            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            wall.setPosition(0);

        }
        else if(intakeOut) {
            intake1.setPower(-intakePower);
            intake2.setPower(intakePower);
            wall.setPosition(0);
        }
        else if (xPress) {
            if (servRot.getColors() == 1){
                servRot.sort(servRot.getPos(),pattern);
                sorted = true;
            }
            else if (!sorted){
                servRot.regRot(servRot.getPos());
            }
        }
        else if(shootActive){
            if (!shooting){
                shooting = true;
                isShoot = true;
                shootingTimer.reset();
            }
            if(shootingTimer.milliseconds() > 500 ) {
                wall.setPosition(0.5);

                intake1.setPower(1);
                intake2.setPower(-1);

                if (shootTimer.milliseconds() > 400) {
                    servRot.fastRot(servRot.getPos());
                    shooterHood.setPosition(shooterHood.getPosition() - recoil);
                    shootTimer.reset();
                }
            }
            else{
                wall.setPosition(0);
            }
            if(shoot1300){
                targetVelocity = 1300;
            }
            power = shooterPIDControl(targetVelocity, currentVelocity);
            shoot1.setPower(-power);
            shoot2.setPower(power);

        }
        else{
            intake1.setPower(0);
            isShoot = false;
            intake2.setPower(0);
            shooting = false;
            //servRot.sSP(0,0);
            shootTimer.reset();
            wall.setPosition(0);
        }

    }

    public double shooterPIDControl(double reference, double state){ //current velocity, target velocity
        //Hey guys, the real PID is actually target velocity - current velocity
        double error=reference-state;
        double dt = PIDTimer.seconds();
        Integralsum+=error*dt;
        double derivative=(error-lasterror)/dt;
        lasterror=error;
        PIDTimer.reset();
        return (error*Kp)+(derivative*Kd)+(Integralsum*Ki)+(reference*Kf);
    }


    public void resetPID(){
        Integralsum=0;
        lasterror=0;
    }

    public double getTurretPos(){
        return shootThing.getTurretPos();
    }

}