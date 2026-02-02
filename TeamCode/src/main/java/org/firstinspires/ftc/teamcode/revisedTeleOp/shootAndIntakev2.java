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

    //axonRotator
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
    double power = 0.0;
    boolean index1 = false;
    boolean isShoot = false;
    double lasterror = 0;
    ElapsedTime PIDTimer = new ElapsedTime();
    double TargetVelocity;
    ElapsedTime shootTimer = new ElapsedTime();
    double currentVelocity,targetVelocity;
    double shootPower;

    ElapsedTime intakeTimer = new ElapsedTime();
    final double intakePower = 0.75; //TODO: find actual power
    double offset  = 400/360*2/5 * 360/355 * 20/18;
    double gearOff = 360/355 * 20/18;

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

        PIDTimer.reset();
        shootTimer.reset();
        intakeTimer.reset();
    }

    //some of the stuff was for pedro so i made it an exclusively teleop class
    public void update(double distance, boolean intakeActive, boolean shootActive, boolean intakeOut,boolean servoReset, Telemetry telemetry){
        //new:
        double intakeDis = servRot.getDisMain();
        if(servoReset){
            servRot.sSP(0,0);
            index1 = false;
        }
        currentVelocity=shoot2.getVelocity();
        targetVelocity=getGoodShootVel(distance);
        power = shooterPIDControl(targetVelocity, currentVelocity)/2;
        isShoot = true;
        shoot1.setPower(-power);
        shoot2.setPower(power);
        if(intakeActive){
            artifactPush.setPosition(kickZero);
            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            wall.setPosition(.5);
            if(intakeDis < 10 && intakeTimer.milliseconds() > 250) {
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
            intake1.setPower(1);
            intake2.setPower(-1);
            wall.setPosition(0);
            if(shootTimer.milliseconds() > 400){
                servRot.regRot(servRot.getPos());
                shootTimer.reset();
            }
            currentVelocity=shoot2.getVelocity();
            targetVelocity=getGoodShootVel(distance);
            power = shooterPIDControl(targetVelocity, currentVelocity);
            shoot1.setPower(-power);
            shoot2.setPower(power);
            if(Math.abs(shoot1.getPower()) < 0.78 * power){
                artifactPush.setPosition(kickZero);
            }
            else{
                artifactPush.setPosition(kickUp);
            }

        }
        else{
            intake1.setPower(0);
            isShoot = false;
            intake2.setPower(0);
            artifactPush.setPosition(kickUp);
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
    public double getGoodShootVel(double distanceFromGoal){
        return 0.0000145 * Math.pow(distanceFromGoal, 4) - 0.00584813 * Math.pow(distanceFromGoal, 3) + 0.834897 * Math.pow(distanceFromGoal, 2) - 45.38315 * Math.pow(distanceFromGoal, 1) + 2000.07059;
    }
    public double hoodPosSet(double distanceFromGoal){
        return  -Math.pow(10, -9) * 2.0571 * Math.pow(distanceFromGoal, 4) - Math.pow(10, -7)*8.57305 * Math.pow(distanceFromGoal, 3) + 0.000313995 * Math.pow(distanceFromGoal, 2) - 0.0237158 * Math.pow(distanceFromGoal, 1) + 0.93;
    }
    public double getRecoil(double distanceFromGoal){
        return  -Math.pow(10, -9) * 5.66719 * Math.pow(distanceFromGoal, 4) + 0.00000199279 * Math.pow(distanceFromGoal, 3) -0.00024284 * Math.pow(distanceFromGoal, 2) +0.0127555 * Math.pow(distanceFromGoal, 1) -0.233045;
    }

    public void resetPID(){
        Integralsum=0;
        lasterror=0;
    }
}