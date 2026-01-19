package org.firstinspires.ftc.teamcode.revisedTeleOp;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ServoRotate;
import org.firstinspires.ftc.teamcode.colorShootFunc;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

//neater colorShootFunc with a few extra things
public class ShooterAndIntake {
    //drivetrain
    Drivetrain drivetrain;
    //intake
    DcMotorEx intake1;
    DcMotorEx intake2;
    Servo intakeGate;

    //both
    Servo artifactPush;
    private final double kickZero = 0.85;
    private final double kickUp = 0.70;

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
    private final double TURRET_START = 0.96;

    NormalizedColorSensor colorSensor;

    final int GREEN = 1;
    final int PURPLE = 2;

    private int pattern[] = {PURPLE, PURPLE, GREEN};
    private int[] spindexColors = new int[3];

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

    private colorShootFunc ColorShootFunc;
    private ElapsedTime servoRotate = new ElapsedTime();

    public ShooterAndIntake(HardwareMap hardwareMap){
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
        //shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 3, 0, 12, MotorControlAlgorithm.PIDF));
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 3, 0, 12, MotorControlAlgorithm.PIDF));
        shooterHood = hardwareMap.get(Servo.class, "shooterHood");
        shooterHood.setPosition(0.4);
        artifactPush.setPosition(kickZero);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "coloora");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "disDiss");

        PIDTimer.reset();
        shootTimer.reset();
        intakeTimer.reset();

        ColorShootFunc = new colorShootFunc(hardwareMap, servRot, shoot1, shoot2, colorSensor, intake1, intake2, wall, artifactPush);
        //, DcMotorEx
    }

    public void setPattern(int pat){
        if(pat == 1){ //23
            pattern[0] = PURPLE;
            pattern[1] = PURPLE;
            pattern[2] = GREEN;
        }
        else if(pat == 2){ //22
            pattern[0] = PURPLE;
            pattern[1] = GREEN;
            pattern[2] = PURPLE;
        }
        else if(pat == 3){ //21
            pattern[0] = GREEN;
            pattern[1] = PURPLE;
            pattern[2] = PURPLE;
        }
    }
    public int[] getPattern(){
        return pattern;
    }

    //some of the stuff was for pedro so i made it an exclusively teleop class
    public void update(double distance, boolean intakeActive, boolean shootActive, boolean intakeOut, boolean colorShootActive, boolean shootGreen, boolean shootPurple, boolean servoReset, boolean servoSpin, boolean kickSwitch, Telemetry telemetry){
        //new:
        ColorShootFunc.wall.setPosition(.15);
        shooterHood.setPosition(.52);
        double intakeDis = distanceSensor.getDistance(DistanceUnit.INCH);
        if(servoReset){
            servRot.servo.setPosition(0);
            servRot.servo2.setPosition(0);
            index1 = false;
        }
        if(kickSwitch){
            if(abs(artifactPush.getPosition() - kickUp)<0.01){
                artifactPush.setPosition(kickZero);
            }
            else if(abs(artifactPush.getPosition() - kickZero)<0.01){
                artifactPush.setPosition(kickUp);
            }
        }
        if(servoSpin){
            ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(), 120, 360);

        }
        if(intakeActive){
            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            ColorShootFunc.wall.setPosition(.5);
            //telemetry.addData("intakeTimer", intakeTimer.milliseconds());
            if(intakeDis < 4 && intakeTimer.milliseconds() > 300) {
                if(index1 && ColorShootFunc.servRo.getPosition() > .39 * gearOff){
                    //ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(), 0, 360);
                    ColorShootFunc.servRo.servo.setPosition(0);
                    ColorShootFunc.servRo.servo2.setPosition(0);
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
            //shoot1.setPower(0);
            //shoot2.setPower(0);
            shoot1.setVelocity(-900);
            shoot2.setVelocity(900);
        }
        else if(intakeOut){
            intake1.setPower(-intakePower);
            intake2.setPower(intakePower);
            //shoot1.setPower(0);
            //shoot2.setPower(0);
            shoot1.setVelocity(-900);
            shoot2.setVelocity(900);
        }
        else if(shootGreen){
            ColorShootFunc.shootOneGreen();
            if(intakeTimer.milliseconds() > 400){
                if( ColorShootFunc.servRo.getPosition() > .39 * gearOff + offset) {
                    ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(), 0, 400);
                }
                else{
                    ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(),120, 400);
                }
                intakeTimer.reset();
            }
        }
        else if(shootPurple){
            ColorShootFunc.shootOnePurple();
            if(intakeTimer.milliseconds() > 400){
                if( ColorShootFunc.servRo.getPosition() > .39 * gearOff + offset) {
                    ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(), 0, 400);
                }
                else{
                    ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(),120, 400);
                }
                intakeTimer.reset();
            }
        }
        else if(shootActive){
           /* if (!index1){
                index1 = true;
                ColorShootFunc.servRo.servo.setPosition(0);
                ColorShootFunc.servRo.servo2.setPosition(0);
            }*/

            currentVelocity=shoot2.getVelocity();
            targetVelocity=getGoodShootVel(distance);
            //targetVelocity=1200;
            //shootPower = shooterPIDControl(targetVelocity, currentVelocity);
            //shootPower = targetVelocity;

            /*if(ColorShootFunc.shootOneBall() == 1){
                if(servRo.getPosition() > .399 * gearOff + offset) {
                    servRo.startRotate(servRo.getPosition(), 0, 405);
                }
                else{
                servRo.startRotate(servRo.getPosition(),120, 405);
                }
            } */
            shoot1.setVelocity(-targetVelocity);
            shoot2.setVelocity(targetVelocity);
            //shoot1.setPower(-shootPower);
            //shoot2.setPower(shootPower);
            intake1.setPower(1);
            intake2.setPower(-1);
            //if((shoot2.getVelocity() > (.98*targetVelocity)) && (shoot2.getVelocity()<(1.02*targetVelocity))){
            if((abs(shoot2.getVelocity()) > (targetVelocity*.85))||(abs(shoot1.getVelocity()) > (targetVelocity*.85))){
                wall.setPosition(0);
                artifactPush.setPosition(kickUp);
                telemetry.addData("in loop", 0);
                if(intakeTimer.milliseconds() > 300){
                    telemetry.addData("in timer", 0);
                    if(!index1){
                        ColorShootFunc.servRo.servo.setPosition(0);
                        ColorShootFunc.servRo.servo2.setPosition(0);
                        ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(), 0, 400);
                        index1 = true;
                        telemetry.addData("in 1", 0);
                    }
                    else if( ColorShootFunc.servRo.getPosition() > .39 * gearOff) {
                        ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(), 0, 400);
                        telemetry.addData("in 2", 0);
                    }
                    else{
                        ColorShootFunc.servRo.startRotate(ColorShootFunc.servRo.getPosition(),120, 400);
                        telemetry.addData("in 3", 0);
                    }
                    intakeTimer.reset();
                }
            }
            else {
                telemetry.addData("in else", 0);
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
    }*/     telemetry.addData("distance",distance);
            telemetry.addData("targetVelocity",targetVelocity);
            telemetry.addData("currentVelocity",currentVelocity);
            telemetry.addData("shootingVelocity",shootPower);
            telemetry.addData("servo1 pos", ColorShootFunc.servRo.servo.getPosition());
            telemetry.addData("servo2 pos", ColorShootFunc.servRo.servo2.getPosition());
            telemetry.update();
        }
        else if(colorShootActive){
            ColorShootFunc.update(distance, intake1.getPower(), intakeDis, Integralsum, lasterror, 1, telemetry, 1);
            stage = ColorShootFunc.score(pattern, stage);
            if(stage == 0){
                stage = 2;
            }
        }
        else{
            //ColorShootFunc.update(distance, intake1.getPower(), intakeDis, Integralsum, lasterror, 1, telemetry, 0);
            //shoot1.setPower(0);
            //shoot2.setPower(0);
            shoot1.setVelocity(-900);
            shoot2.setVelocity(900);
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
        return 0.0000189394*(dis*dis*dis*dis) - 0.00598485*(dis*dis*dis) + 0.70947*(dis*dis) - 34.90476*(dis) + 1750.01299;
    }

    public void resetPID(){

        Integralsum=0;
        lasterror=0;
    }

}

