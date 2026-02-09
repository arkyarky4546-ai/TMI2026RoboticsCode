package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.servo720Rot;

//Organized by Johnson

public class ShooterAndIntakeClean {
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
    public static double Kp=0.0012;
    public static double Ki=0.0000;
    public static double Kd=0.0001;
    public static double Kf=.0006;
    boolean index1 = false;
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

    public ShooterAndIntakeClean(HardwareMap hardwareMap){
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
        if(intakeActive){
            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            wall.setPosition(.5);
            if(intakeDis < 10 && intakeTimer.milliseconds() > 150) {
                servRot.regRot(servRot.getPos());
                intakeTimer.reset();
            }
            shoot1.setVelocity(-0);
            shoot2.setVelocity(0);
        }
        else if(intakeOut) {
            intake1.setPower(-intakePower);
            intake2.setPower(intakePower);
            shoot1.setVelocity(-0);
            shoot2.setVelocity(0);
        }
        else if(shootActive){
            currentVelocity=shoot2.getVelocity();
            targetVelocity=getGoodShootVel(distance);
            shoot1.setVelocity(-targetVelocity);
            shoot2.setVelocity(targetVelocity);
            intake1.setPower(1);
            intake2.setPower(-1);
            if((Math.abs(shoot2.getVelocity()) > (targetVelocity*.85))||(Math.abs(shoot1.getVelocity()) > (targetVelocity*.85))){
                wall.setPosition(0);
                artifactPush.setPosition(kickUp);
                telemetry.addData("in loop", 0);
                if(intakeTimer.milliseconds() > 300){
                    telemetry.addData("in timer", 0);
                    servRot.regRot(servRot.getPos());
                    intakeTimer.reset();
                }
            }
            else {
                telemetry.addData("in else", 0);
                wall.setPosition(.3);
                artifactPush.setPosition(kickZero);
            }
            telemetry.addData("distance",distance);
            telemetry.addData("targetVelocity",targetVelocity);
            telemetry.addData("currentVelocity",currentVelocity);
            telemetry.addData("shootingVelocity",shootPower);
            telemetry.addData("servo1 pos", servRot.getPos());
            telemetry.addData("servo2 pos", servRot.getPos());
            telemetry.update();
        }
        else{
            shoot1.setVelocity(-0);
            shoot2.setVelocity(0);
            intake1.setPower(0);
            intake2.setPower(0);
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
        return 0.0000145 * Math.pow(distanceFromGoal, 4) - 0.00584813 * Math.pow(distanceFromGoal, 3) + 0.834897 * Math.pow(distanceFromGoal, 2) - 45.38315 * Math.pow(distanceFromGoal, 1) + 2020.07059;
    }
    public double hoodPosSet(double distanceFromGoal){
        return  -Math.pow(10, -9) * 2.0571 * Math.pow(distanceFromGoal, 4) - Math.pow(10, -7)*8.57305 * Math.pow(distanceFromGoal, 3) + 0.000313995 * Math.pow(distanceFromGoal, 2) - 0.0237158 * Math.pow(distanceFromGoal, 1) + 0.862228;
    }
    public double getRecoil(double distanceFromGoal){
        return  -Math.pow(10, -9) * 5.66719 * Math.pow(distanceFromGoal, 4) + 0.00000199279 * Math.pow(distanceFromGoal, 3) -0.00024284 * Math.pow(distanceFromGoal, 2) +0.0127555 * Math.pow(distanceFromGoal, 1) -0.233045;
    }

    public void resetPID(){
        Integralsum=0;
        lasterror=0;
    }
}