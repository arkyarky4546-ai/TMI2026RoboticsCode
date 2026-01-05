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

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime timer12 = new ElapsedTime();
    ElapsedTime timer123 = new ElapsedTime();
    ElapsedTime timer1234 = new ElapsedTime();
    double lasterror = 0;
    Servo wally;
    int coolor;
    float time = System.currentTimeMillis();
    private int[] spindexColors = new int[3];
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
    DcMotor inta1;
    DcMotor inta2;
    int scanPos = 0;
    int green = 1;
    int purple = 2;
    double TargetVelocity;
    double kickZero = 0.85;
    double kickUp = 0.75;
    Servo wall;
    Servo artiPush;
    double shootPower;
    int i = 0;

    public colorShootFunc(HardwareMap hardwareMap, ServoRotate servoRot, DcMotorEx shoot1, DcMotorEx shoot2, NormalizedColorSensor coloora, DcMotor intake1, DcMotor intake2, Servo wally, Servo ArtifactPush) {
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
    public void update( double distance, double power, double dis, double integralsum, double Lasterror, int pathstate){
        if(pathstate == 12) {
            shootPower = 0;
        }
        else {
            shootPower = PIDControl(shoot2.getVelocity(), getGoodVel(distance));
        }
        Integralsum = integralsum;
        lasterror = Lasterror;
        inta2.setPower(power);
        inta2.setPower(-power);
        shooting1.setPower(shootPower);
        shooting2.setPower(-shootPower);
        if( dis < 5 && timer123.milliseconds() > 100){
            servRo.startRotate(servRo.getPosition() , 120, 0);
            timer123.reset();
        }
        scOOON();
    }
    public double getGoodVel(double dis){
        return -217*(dis*dis*dis) + 875.6403*(dis*dis) -1196.11498*(dis) + 1830.8098;
    }
    public int score(int[] pattern, int stage){
        if( spindexColors[i] == pattern[stage]){
            artiPush.setPosition(kickUp);
            shootOneBall();
            spindexColors[i] = 0;
            i+=1;
            return (stage-1);
        }
        else{
            artiPush.setPosition(kickZero);
            i+=1;
            servRo.startRotate(servRo.getPosition(), 120, 0);
            return stage;
        }

    }
    public int scOOON () {
        if (scanPos < 3 && timer12.milliseconds() > 100){
            scanPos++;
            timer12.reset();
            servRo.startRotate(servRo.getPosition(), 120, 0);
            spindexColors[scanPos] = getColors();
        }
        return scanPos;
    }

    public int shootOneBall(){ //wasn't letting me use thread.sleep for some reason so it told me to add this
        wall.setPosition(0.3);
        Integralsum = 0;
        lasterror = 0;
        TargetVelocity = getGoodVel(shooting2.getVelocity());
        if(shooting2.getVelocity()>(.92*TargetVelocity)){
            wall.setPosition(0);
            artiPush.setPosition(kickUp);

            //artifactPush.setPosition(kickZero);
            timer1234.reset();
            return 1;

            //gateShoot = false;
        }
        else {
            wall.setPosition(.3);
            return 0;
        }
    }
}
