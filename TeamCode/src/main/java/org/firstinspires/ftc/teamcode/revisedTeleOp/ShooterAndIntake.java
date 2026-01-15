package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ServoRotate;
import org.firstinspires.ftc.teamcode.colorShootFunc;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

//neater colorShootFunc with a few extra things
public class ShooterAndIntake {
    //intake
    DcMotorEx intake1;
    DcMotorEx intake2;
    Servo intakeGate;

    //both
    Servo artifactPush;
    private final double kickZero = 0.85;
    private final double kickUp = 0.75;

    //axonRotator
    ServoRotate servRo;
    //CRServo slave;

    //shooter
    Servo turretRight;
    Servo turretLeft;
    DcMotorEx shoot1;
    DcMotorEx shoot2;
    Servo wall;
    Servo shooterHood;
    private final double TURRET_START = 0.5;
    private double shooterPower;

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
    double lasterror = 0;
    ElapsedTime PIDTimer = new ElapsedTime();
    double TargetVelocity;
    ElapsedTime shootTimer = new ElapsedTime();
    double shootPower;

    ElapsedTime intakeTimer = new ElapsedTime();
    final double intakePower = 0.75; //TODO: find actual power
    int stage = 2;
    DistanceSensor distanceSensor;
    double offset  = 405/360*2/5 * 360/355 * 20/18;
    double gearOff = 360/355 * 20/18;

    private colorShootFunc ColorShootFunc;
    private ElapsedTime servoRotate = new ElapsedTime();

    public ShooterAndIntake(HardwareMap hardwareMap){
        intake1 = hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake1");
        intakeGate = hardwareMap.get(Servo.class, "intakeGate");
        intakeGate.setPosition(0);

        artifactPush = hardwareMap.get(Servo.class, "push");

        servRo = new ServoRotate(hardwareMap,"spindexRoter", "slave");
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
        shooterPower = 1;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "coloora");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "disDiss");

        PIDTimer.reset();
        shootTimer.reset();
        intakeTimer.reset();

        ColorShootFunc = new colorShootFunc(hardwareMap, servRo, shoot1, shoot2, colorSensor, intake1, intake2, wall, artifactPush);
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
    public void update(double distance, boolean intakeActive, boolean shootActive, boolean intakeOut, boolean colorShootActive, boolean shootGreen, boolean shootPurple, Telemetry telemetry){
        //new:
        double intakeDis = distanceSensor.getDistance(DistanceUnit.INCH);
        if(intakeActive){
            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            //telemetry.addData("intakeTimer", intakeTimer.milliseconds());
            if(intakeDis < 5 && intakeTimer.milliseconds() > 300) {
                servRo.startRotate(servRo.getPosition(), 120, 0);
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
        else if(shootGreen){
            ColorShootFunc.shootOneGreen();
            servRo.startRotate(servRo.getPosition(),120, 405);
        }
        else if(shootPurple){
            ColorShootFunc.shootOnePurple();
            servRo.startRotate(servRo.getPosition(),120, 405);
        }
        else if(shootActive){
            shootPower = shooterPIDControl(shoot2.getVelocity(), getGoodShootVel(distance));
            shootPower = 1300;
            /*if(ColorShootFunc.shootOneBall() == 1){
                if(servRo.getPosition() > .399 * gearOff + offset) {
                    servRo.startRotate(servRo.getPosition(), 0, 405);
                }
                else{
                servRo.startRotate(servRo.getPosition(),120, 405);
                }
            } */
            shoot1.setPower(-shootPower);
            shoot2.setPower(shootPower);
            if(shoot2.getVelocity()>(.92*shootPower)){
                wall.setPosition(0);
                artifactPush.setPosition(kickUp);
                if(intakeTimer.milliseconds() > 300){
                    servRo.startRotate(servRo.getPosition(),120, 405);
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
    }*/

        }
        else if(colorShootActive){
            ColorShootFunc.update(distance, intake1.getPower(), intakeDis, Integralsum, lasterror, 1, telemetry);
            stage = ColorShootFunc.score(pattern, stage);
            if(stage == 0){
                stage = 2;
            }
        }
        else{
            ColorShootFunc.update(distance, intake1.getPower(), intakeDis, Integralsum, lasterror, 1, telemetry);
            shoot1.setPower(0);
            shoot2.setPower(0);
            intake1.setPower(0);
            intake2.setPower(0);
        }

    }

    public double shooterPIDControl(double reference, double state){ //current velocity, target velocity
        double error=reference-state;
        double dt = PIDTimer.seconds();
        Integralsum+=error*dt;
        double derivative=(error-lasterror)/dt;
        lasterror=error;
        PIDTimer.reset();
        return (error*Kp)+(derivative*Kd)+(Integralsum*Ki)+(reference*Kf);
    }

    public double getGoodShootVel(double dis){
        return -217*(dis*dis*dis) + 875.6403*(dis*dis) -1196.11498*(dis) + 1830.8098;
    }


}

