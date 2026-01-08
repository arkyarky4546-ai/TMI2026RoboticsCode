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

//neater colorShootFunc with a few extra things
public class ShooterAndIntake {
    //intake
    DcMotor intake1;
    DcMotor intake2;
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

    int scanPos;
    int currentPos;
    ElapsedTime scanTimer = new ElapsedTime();

    boolean offsetForScoring = true;

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

    int shootTrack = 1;

    public ShooterAndIntake(HardwareMap hardwareMap){
        intake1 = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake1");
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

        currentPos = 0;

        scanTimer.reset();
        PIDTimer.reset();
        shootTimer.reset();
        intakeTimer.reset();
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
    public void update(double distance, boolean intakeActive, boolean shootActive, boolean intakeOut){
        //new:
        if(intakeActive){
            double intakeDis = distanceSensor.getDistance(DistanceUnit.INCH);
            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            if(intakeDis < 5 && intakeTimer.milliseconds() > 300) {
                servRo.startRotate(servRo.getPosition(), 120, 0);
                currentPos += 1;
                if (currentPos == 3) {
                    currentPos = 0;
                }
                intakeTimer.reset();
            }
        }
        else if(intakeOut){
            intake1.setPower(-intakePower);
            intake2.setPower(intakePower);
        }
        else if(shootActive){
            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            shootPower = shooterPIDControl(shoot2.getVelocity(), getGoodShootVel(distance));
            stage = score(stage);
            if(stage == 0){
                stage = 2;
            }
        }
        else{
            intake1.setPower(0);
            intake2.setPower(0);
            scanAll();
        }

    }

    public int getColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        int color = 0;
        if (colors.red <= .001 && colors.green <= .001 && colors.blue <= .001){
            return 0;
        }
        else if (colors.green > colors.blue && colors.green > .001){
            return GREEN;
        }
        else if (colors.blue > colors.green && colors.blue > .001){
            return PURPLE;
        }

        return color;
    }

    public int scanAll () {
        if (scanPos < 3 && scanTimer.milliseconds() > 1000){
            if(offsetForScoring){
                servRo.startRotate(servRo.getPosition(), 30, 0);
                offsetForScoring = false;
            }
            scanTimer.reset();
            if (servRo.getPosition() >= .39){
                servRo.startRotate(servRo.getPosition(), 0, 0);
            }
            else {
                servRo.startRotate(servRo.getPosition(), 120, 0);
            }
            spindexColors[scanPos] = getColor();
            currentPos++;
            if( currentPos >= 3){
                currentPos = 0;
            }
            scanPos++;
        }
        return scanPos;
    }
    public void reset () {
        scanPos = 0;
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

    public int shootOneBall(){ //wasn't letting me use thread.sleep for some reason so it told me to add this
        wall.setPosition(0.3);
        Integralsum = 0;
        lasterror = 0;
        TargetVelocity = getGoodShootVel(shoot2.getVelocity());
        if(TargetVelocity > 1500){
            TargetVelocity = 1100;
        }
        if(shoot2.getVelocity()>(.92*TargetVelocity)){
            wall.setPosition(0);
            artifactPush.setPosition(kickUp);

            //artifactPush.setPosition(kickZero);
            shootTimer.reset();
            return 1;

            //gateShoot = false;
        }
        else {
            wall.setPosition(.3);
            return 0;
        }
    }

    public int score(int stage){
        if( !offsetForScoring ){
            offsetForScoring = true;
            servRo.startRotate(servRo.getPosition(), 0, 60);
            return stage;

        }
        else if( spindexColors[currentPos] == pattern[stage] && shootTrack == 1){
            artifactPush.setPosition(kickUp);
            shootTrack = shootOneBall();
            spindexColors[currentPos] = 0;

            currentPos+=1;
            if (currentPos==3){
                currentPos=0;
            }
            if (stage == 0){
                return 0;
            }
            else {
                return (stage - 1);
            }
        }
        else if (shootTrack == 1){
            artifactPush.setPosition(kickZero);
            currentPos += 1;
            if (currentPos==3){
                currentPos=0;
            }

            if (servRo.getPosition() >= .39) {
                servRo.startRotate(servRo.getPosition(), 0, 60);
            } else {
                servRo.startRotate(servRo.getPosition(), 120, 60);
            }

            return stage;
        }
        else{
            return stage;
        }

    }

}

