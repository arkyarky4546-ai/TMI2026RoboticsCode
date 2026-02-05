package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.servo720Rot;

//Organized by Johnson

public class shootAndIntakev2 {
    //intake
    DcMotorEx intake1;
    DcMotorEx intake2;
    Servo intakeGate;

    //both
    Servo artifactPush;
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
    private double targetVelocity;

    private double recoil;
    private double intakeDis = 0.0;
    double power = 0.0;
    boolean index1 = false;
    boolean isShoot = false;
    double lasterror = 0;
    ElapsedTime PIDTimer = new ElapsedTime();
    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime updateTimer = new ElapsedTime();
    ElapsedTime intakeUpdate = new ElapsedTime();
    final double intakePower = 1.0; //TODO: find actual power
    double offset  = 400/360*2/5 * 360/355 * 20/18;
    double gearOff = 360/355 * 20/18;
    private regressCalc regression;
    private sensCalc sensors;

    public shootAndIntakev2(HardwareMap hardwareMap){
        intake1 = hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake1");
        intakeGate = hardwareMap.get(Servo.class, "intakeGate");
        intakeGate.setPosition(0);

        artifactPush = hardwareMap.get(Servo.class, "push");

        servRot = new servo720Rot(hardwareMap,"spindexRoter", "slave","disDiss","dis2","dis3","dis4","dis5","dis6","dis7");

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
        artifactPush.setPosition(kickZero);
        regression = new regressCalc();
        sensors = new sensCalc(servRot);
        PIDTimer.reset();
        shootTimer.reset();
        regression.start();
        sensors.start();
    }
    public void stopT(){
        sensors.stopThread();
        regression.stopThread();
    }
    //some of the stuff was for pedro so i made it an exclusively teleop class
    public void update(double distance, boolean intakeActive, boolean shootActive, boolean intakeOut,boolean servoReset, Telemetry telemetry){
        if(servoReset){
            servRot.sSP(0,0);
            index1 = false;
        }
        if(updateTimer.milliseconds() > 30) {
            regression.setDis(distance);
            currentVelocity = shoot2.getVelocity();
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
            isShoot = false;
            artifactPush.setPosition(kickZero);
            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            wall.setPosition(.5);
            if(intakeDis < 10 && intakeTimer.milliseconds() > 290 && !isShoot) {
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
            artifactPush.setPosition(kickUp);
            isShoot = true;
            intake1.setPower(1);
            intake2.setPower(-1);
            wall.setPosition(0);
            if(shootTimer.milliseconds() > 400){
                servRot.regRot(servRot.getPos());
                shooterHood.setPosition(shooterHood.getPosition() - recoil);
                shootTimer.reset();
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
            //servRot.sSP(0,0);
            shootTimer.reset();
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
}