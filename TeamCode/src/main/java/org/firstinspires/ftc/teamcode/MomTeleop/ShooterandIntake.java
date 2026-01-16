/*package org.firstinspires.ftc.teamcode.MomTeleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ServoRotate;


public class ShooterandIntake {
    //drivetrain
    Drivetrain drivetrain;
    //intake
    DcMotorEx intake1;
    DcMotorEx intake2;
    Servo intakeGate;

    //both
    Servo artifactPush;
    private final double kickZero = 0.85;
    private final double kickUp = 0.75;

    //axonRotator
    ServoRotate servRot;
    //CRServo slave;

    //shooter
    Servo turretRight;
    Servo turretLeft;
    DcMotorEx shoot1;
    DcMotorEx shoot2;
    Servo wall;
    Servo shooterHood;
    private final double TURRET_START = 0.5;

    double Integralsum = 0;
    public static double Kp=0.0047;
    public static double Ki=0.0004;
    public static double Kd=0;
    public static double Kf=0;
    boolean index1 = false;
    double lasterror = 0;
    ElapsedTime PIDTimer = new ElapsedTime();
    double TargetVelocity;
    ElapsedTime shootTimer = new ElapsedTime();
    double currentVelocity,targetVelocity;
    double shootPower;

    ElapsedTime intakeTimer = new ElapsedTime();
    final double intakePower = 0.75; //TODO: find actual power
    int stage = 2;
    DistanceSensor distanceSensor;
    double offset  = 400/360*2/5 * 360/355 * 20/18;
    double gearOff = 360/355 * 20/18;

    private ElapsedTime servoRotate = new ElapsedTime();

    public ShooterandIntake(HardwareMap hardwareMap){
        drivetrain = new Drivetrain(hardwareMap);

        intake1 = hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake1");
        intakeGate = hardwareMap.get(Servo.class, "intakeGate");
        intakeGate.setPosition(0);

        artifactPush = hardwareMap.get(Servo.class, "push");

        servRot = new ServoRotate(hardwareMap,"spindexRoter", "slave");
        //slave = hardwareMap.get(CRServo.class, "slave"); do we really need this?

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

        distanceSensor = hardwareMap.get(DistanceSensor.class, "disDiss");

        PIDTimer.reset();
        shootTimer.reset();
        intakeTimer.reset();

    }


    //some of the stuff was for pedro so i made it an exclusively teleop class
    public void update(double distance, boolean intakeActive, boolean shootActive, boolean intakeOut, boolean colorShootActive, boolean shootGreen, boolean shootPurple, Telemetry telemetry){
        //new:
        double intakeDis = distanceSensor.getDistance(DistanceUnit.INCH);
        if(intakeActive){
            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            ColorShootFunc.wall.setPosition(.5);
            //telemetry.addData("intakeTimer", intakeTimer.milliseconds());
            if(intakeDis < 5 && intakeTimer.milliseconds() > 300) {
                if(index1 && ColorShootFunc.servRo.getPosition() > .39 * gearOff){
                    ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(), 0, 360);
                    index1 = false;
                }
                else if( ColorShootFunc.servRo.getPosition() > .39 * gearOff) {
                    ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(), 0, 360);

                }
                else{
                    ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(),120, 360);
                }
                intakeTimer.reset();
            }
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        else if(intakeOut){
            intake1.setPower(-intakePower);
            intake2.setPower(intakePower);
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        else if(shootActive){
            currentVelocity=shoot2.getVelocity();
            targetVelocity=getGoodShootVel(distance);
            shootPower = shooterPIDControl(targetVelocity, currentVelocity);

            /*if(ColorShootFunc.shootOneBall() == 1){
                if(servRo.getPosition() > .399 * gearOff + offset) {
                    servRo.startRotate(servRo.getPosition(), 0, 405);
                }
                else{
                servRo.startRotate(servRo.getPosition(),120, 405);
                }
            } */
           /* shoot1.setPower(-shootPower);
            shoot2.setPower(shootPower);
            intake1.setPower(1);
            intake2.setPower(-1);
            //telemetry.addData("shjoot power", shoot2.getVelocity());
            if((shoot2.getVelocity() > (.95*targetVelocity))&&(shoot2.getVelocity()<(1.05*targetVelocity))){
                wall.setPosition(0);
                artifactPush.setPosition(kickUp);
                if(intakeTimer.milliseconds() > 300){
                    if(!index1 && ColorShootFunc.servRo.getPosition() > .39 * gearOff){
                        ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(), 0, 360);
                        ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(), 0, 400);
                        index1 = true;
                    }
                    else if( ColorShootFunc.servRo.getPosition() > .39 * gearOff + offset) {
                        ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(), 0, 400);

                    }
                    else{
                        ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(),120, 400);
                    }
                    intakeTimer.reset();
                }
            }
            else {
                wall.setPosition(.3);
                artifactPush.setPosition(kickZero);
            }

            /*wall.setPosition(0.3);
        Integralsum = 0;
        lasterror = 0;
        TargetVelocity = getGoodVel(shooting2.getVelocity());
        if(TargetVelocity > 1500){
            TargetVelocity = 1300;
        }
        if(shooting2.getVelocity()>(.92*TargetVelocity)){
            wall.setPosition(0);
            artiPush.setPosition(kickUp);

            //artifactPush.setPosition(kickZero);
           // timer1234.reset();
            return 1;

            //gateShoot = false;
        }
        else {
            wall.setPosition(.3);
            return 0;
        }
    }     telemetry.addData("distance",distance);
            telemetry.addData("targetVelocity",targetVelocity);
            telemetry.addData("currentVelocity",currentVelocity);
            telemetry.addData("shootingVelocity",shootPower);
            telemetry.update();
        }
        else{
            ColorShootFunc.update(distance, intake1.getPower(), intakeDis, Integralsum, lasterror, 1, telemetry, 0);
            shoot1.setPower(0);
            shoot2.setPower(0);
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

    public double getGoodShootVel(double dis){
        return 0.0000189394*(dis*dis*dis*dis) - 0.00598485*(dis*dis*dis) + 0.70947*(dis*dis) - 34.90476*(dis) + 1687.01299;
    }
}*/


