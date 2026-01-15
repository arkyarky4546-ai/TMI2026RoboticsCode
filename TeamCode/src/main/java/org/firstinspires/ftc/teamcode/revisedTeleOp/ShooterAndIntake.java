package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ServoRotate;
import org.firstinspires.ftc.teamcode.colorShootFunc;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
    public void update(double distance, boolean intakeActive, boolean shootActive, boolean intakeOut, boolean colorShootActive){
        //new:
        double intakeDis = distanceSensor.getDistance(DistanceUnit.INCH);
        if(intakeActive){
            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            if(intakeDis < 5 && intakeTimer.milliseconds() > 300) {
                servRo.startRotate(servRo.getPosition(), 120, 0);
                intakeTimer.reset();
            }
        }
        else if(intakeOut){
            intake1.setPower(-intakePower);
            intake2.setPower(intakePower);
        }
        else if(shootActive){
            shootPower = shooterPIDControl(shoot2.getVelocity(), getGoodShootVel(distance));
            if(ColorShootFunc.shootOneBall() == 1){
                if(servRo.getPosition() > .399 * gearOff + offset) {
                    servRo.startRotate(servRo.getPosition(), 0, 405);
                }
                else{
                servRo.startRotate(servRo.getPosition(),120, 405);
                }
            }

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

