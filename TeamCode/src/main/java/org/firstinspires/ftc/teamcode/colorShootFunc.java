package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ServoRotate;

public class colorShootFunc {
    private NormalizedColorSensor color;
    CRServo artifactSpinner;
    //Servo artifactPush;
    DcMotorEx shoot1;
    DcMotorEx shoot2;
    DcMotor intakeMotor;
    DcMotor intakeMotor1;
    AxonRotator smart1;
    boolean index1 = false;
    CRServo smart;
    double distance;
    double Integralsum = 0;
    public static double Kp=0.0047;
    public static double Ki=0.0004;
    public static double Kd=0;
    public static double Kf=0;

    boolean indextooffset = false;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime timer12 = new ElapsedTime();
    ElapsedTime timer123 = new ElapsedTime();
    ElapsedTime timer1234 = new ElapsedTime();
    ElapsedTime timer123456 = new ElapsedTime();
    boolean intake = false;
    double lasterror = 0;
    Servo wally;
    int coolor;
    float time = System.currentTimeMillis();
    private int[] spindexColors = new int[3];
    private int[] spindexColors1 = new int[3];
    private int currentScanPos = 0;
    private boolean isScanning = false;
    private ElapsedTime scanTimer = new ElapsedTime();
    boolean sort = false;
    int numpurp = 0;
    int numgreen = 0;
    int pattern;
    DistanceSensor dis;
    //AxonRotator smart2;
    CRServo slave;
    Timer limelightTimer;
    boolean turretToLeft;
    boolean turretToRight;
    int actPattern;
    int in = 1;
    boolean turningLeft = false;
    boolean turningRight = false;
    public ServoRotate servRo;

    DcMotorEx shooting1;
    DcMotorEx shooting2;
    DcMotorEx inta1;
    DcMotorEx inta2;
    int scanPos = 0;
    int green = 1;
    int purple = 2;
    double TargetVelocity;
    double kickZero = 0.85;
    double kickUp = 0.72;
    public Servo wall;
    Servo artiPush;
    double shootPower;
    int i = 0;
    int shootTrack = 0;
    int colorIndex = 0;
    double offset  = 400/360*2/5 * 360/355 * 20/18;
    double gearOff = 360/355 * 20/18;
    double intakeVelocity;
    boolean oppInt = false;
    boolean fstspdachvd = false;

    public colorShootFunc(HardwareMap hardwareMap, ServoRotate servoRot, DcMotorEx shoot1, DcMotorEx shoot2, NormalizedColorSensor coloora, DcMotorEx intake1, DcMotorEx intake2, Servo wally, Servo ArtifactPush) {
        this.servRo  = servoRot;
        this.shooting1 = shoot1;
        this.shooting2 = shoot2;
        this.color = coloora;
        this.inta1 = intake1;
        this.inta2 = intake2;
        this.wall = wally;
        this.artiPush = ArtifactPush;
        timer12.reset();
        timer1234.reset();
        timer123.reset();
        timer1.reset();


    }
    public int getColors(){

        NormalizedRGBA colors = color.getNormalizedColors();
        int color = 0;
        if (colors.red <= .001 && colors.green <= .001 && colors.blue <= .001){
            color = 0;
        }
        else if (colors.green > colors.blue && colors.green > .001){
            color = green;
            numgreen += 1;
        }
        else if (colors.blue > colors.green && colors.blue > .001){
            color = purple;
            numpurp +=1;
        }

        return color;
    }
    public double PIDControl(double reference, double state){
        double error=reference-state;
        double dt = timer.seconds();
        Integralsum+=error*dt;
        double derivative=(error-lasterror)/dt;
        lasterror=error;
        timer.reset();
        return (error*Kp)+(derivative*Kd)+(Integralsum*Ki)+(reference*Kf);
    }
    public int update( double distance, double power, double dis, double integralsum, double Lasterror, int pathstate, Telemetry telemetry, int on){
        if(pathstate == 12 || on == 0) {
            shootPower = 0;
        }
        else {
            shootPower = PIDControl(shooting2.getVelocity(), getGoodVel(distance));
        }
        Integralsum = integralsum;
        lasterror = Lasterror;

        intakeVelocity = -inta1.getVelocity();
        if (!fstspdachvd){
            inta2.setPower(-power);
            inta1.setPower(power);
        }
        /*if (intakeVelocity > 1200){
            fstspdachvd = true;
        }*/
        if (fstspdachvd) {
            if (intakeVelocity < 600 && power != 0 && timer1.milliseconds() > 2000) {
                oppInt = true;
                artiPush.setPosition(kickZero);
                inta2.setPower(power);
                inta1.setPower(-power);
            }
            if (oppInt) {
                oppInt = false;
                timer1.reset();
            } else if (timer1.milliseconds() > 300) {
                inta2.setPower(-power);
                inta1.setPower(power);
            }
        }


        shooting1.setPower(shootPower);
        shooting2.setPower(-shootPower);
        telemetry.addData("color array 1: ", spindexColors[0]);
        telemetry.addData("color array 2: ", spindexColors[1]);
        telemetry.addData("color array 3: ", spindexColors[2]);
        telemetry.addData("intakevel ", inta1.getVelocity());

        if( dis < 5 && timer123.milliseconds() > 200){
            if (servRo.getPosition() >= .399 * gearOff + offset) {
                servRo.startRotate(servRo.getPosition(), 0, 360);
                i = 0;
            } else {
                servRo.startRotate(servRo.getPosition(), 120, 360);
            }
            i += 1;
            if (i == 3){
                i = 0;
            }
           // intake = true;
            timer123.reset();
            telemetry.addData("dis ", dis);
        }
        /*if (intake && timer123.milliseconds() > 75){
            servRo.startRotate(servRo.getPosition() , 80, 0);
            timer123.reset();
            intake = false;
        }*/
        return scOOON();
        //return 0;
    }
    public double getGoodVel(double dis){
        return -217*(dis*dis*dis) + 875.6403*(dis*dis) -1196.11498*(dis) + 1830.8098;
    }
    public int score(int[] pattern, int stage){
        if( !indextooffset ){
            indextooffset = true;
            servRo.servo.setPosition(0);
            i = 0;
            servRo.servo2.setPosition(0);
            servRo.startRotate(servRo.getPosition(), 0, 400);
            return stage;

        }
        else if( spindexColors[i] == pattern[stage] && timer123456.milliseconds() > 500) {
            if (shootTrack == 0) {

                shootTrack = shootOneBall();
                timer123456.reset();
                return stage;
            }

            else if (shootTrack == 1 && timer123456.milliseconds() > 3000) {
                timer123456.reset();
                spindexColors[i] = 0;
                shootTrack = 0;
                artiPush.setPosition(kickZero);

                i += 1;
                if (i == 3) {
                    i = 0;
                }
                if (servRo.getPosition() >= .399 * gearOff + offset) {
                    servRo.startRotate(servRo.getPosition(), 0, 400);
                    i = 0;
                } else {
                    servRo.startRotate(servRo.getPosition(), 120, 400);
                }
                if (stage == 0) {
                    return 0;
                } else {
                    return (stage - 1);
                }
            }
            else{
                return stage;
            }

        }
        else if (spindexColors[i] != pattern[stage] && timer123456.milliseconds() > 3000){
            timer123456.reset();
            artiPush.setPosition(kickZero);
            i += 1;
            if (i==3){
                i=0;
            }

            if (servRo.getPosition() >= .399 * gearOff + offset) {
                servRo.startRotate(servRo.getPosition(), 0, 400);
                i = 0;
            } else {
                servRo.startRotate(servRo.getPosition(), 120, 400);
            }

            return stage;
        }
        else{
            return stage;
        }

       }
    public int scOOON () {

        if (scanPos < 3) {
            wall.setPosition(.5);
            if (timer12.milliseconds() > 1000) {
                spindexColors[scanPos] = getColors();

            /*if(indextooffset){
                servRo.startRotate(servRo.getPosition(), 30, 0);
                indextooffset = false;
            }*/
                i++;
                if (i >= 3) {
                    i = 0;
                }
                timer12.reset();
                if( servRo.getPosition() >= .399 * gearOff){
                    servRo.startRotate(servRo.getPosition(), 0, 360);
                    i = 0;
                }
                else {
                    servRo.startRotate(servRo.getPosition(), 120, 360);
                }



                scanPos++;

            }
        }

        return scanPos;
    }
    public void reset () {
        scanPos = 0;
    }

    public boolean shootOneGreen(){
        if (spindexColors[i] == green) {
            return (shootOneBall() == 1);
        }
        else{
            servRo.startRotate(servRo.getPosition(), 120, 400);
            i--;
            if(i == -1){
                i = 2;
            }
            return false;
        }
    }

    public boolean shootOnePurple(){
        if (spindexColors[i] == purple) {
            return (shootOneBall() == 1);
        }
        else{
            servRo.startRotate(servRo.getPosition(), 120, 400);
            i--;
            if(i == -1){
                i = 2;
            }
            return false;
        }
    }

    public int shootOneBall(){ //wasn't letting me use thread.sleep for some reason so it told me to add this
        wall.setPosition(0.3);
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
    }
}
