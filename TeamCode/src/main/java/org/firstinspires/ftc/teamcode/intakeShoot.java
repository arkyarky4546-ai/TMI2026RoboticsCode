package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.revisedTeleOp.sensCalc;
import org.firstinspires.ftc.teamcode.revisedTeleOp.sensCalc1;
import org.firstinspires.ftc.teamcode.servo720Rot;

public class intakeShoot {

    //DcMotors
    private DcMotorEx intakeMotor1, intakeMotor2, shootMotor1, shootMotor2;

    //my spindex rotator class. look at servo720Rot
    private servo720Rot spindexer;

    private Servo wall;
    //doubles
    //variable that controls shooting power
    private double shootPower;
    private double ticksPerRev = 28;
    private double TargetVelocity = 0;
    private double distance;

    //doubles having to do with PID Tuning (If you dont know PID Tuning look it up, its very useful)
    private double IntegralSum = 0;
    private double lastError = 0;
    public static double Kp=0.0121;
    public static double Ki=0.00014;
    public static double Kd=0.0000;
    public static double Kf=.0000;

    //ints
    private int shootMode = 1;
    private int index;
    private int intakeMode = 0;
    private int arrayShootIntakeTrack;

    //various timers for delaying stuff (super useful in a lot of scenarios)
    private ElapsedTime PIDtimer = new ElapsedTime();
    private ElapsedTime Intaketimer = new ElapsedTime();
    private sensCalc1 sensors;
    private shooterThread Values;
    private Servo hoods;

    public intakeShoot(HardwareMap hardwareMap, String intake1, String intake2, String shoot1, String shoot2, String servoName, String servoName2, String distance1, String distance2, String distance3, String distance4, String distance5, String distance6, String distance7, String wallName, String colorS1, String colorS2, String shooterHood) {
        //constructor this is where everything is initialized
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, intake1);
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, intake2);
        wall = hardwareMap.get(Servo.class, wallName);
        shootMotor1 = hardwareMap.get(DcMotorEx.class, shoot1);
        shootMotor2 = hardwareMap.get(DcMotorEx.class, shoot2);
        hoods = hardwareMap.get(Servo.class, shooterHood);

        //starting all the timers
        PIDtimer.reset();
        Intaketimer.reset();

        //my custom class takes all of these variables
        spindexer = new servo720Rot(hardwareMap, servoName, servoName2, distance1, distance2, distance3, distance4, distance5, distance6, distance7, colorS1, colorS2);
        spindexer.sSP(0,0);
        sensors = new sensCalc1(spindexer);
        sensors.start();
        Values = new shooterThread();
        Values.start();

    }
    //most of the times useful to have an update method to update servo positions or motor powers and other stuff
    public void update(double intakePower, int pathstate, boolean intake){

        if(pathstate == 16) {
            shootPower = 0;
            intakePower = 0;
        }
        else {
            //using PID and a targetVelocity in order to keep the right motor velocity
            shootsetVelocity(Values.getSpeed());

        }
        //setting the power of the shooter and intake here
        shootsetPower(shootPower);
        intakesetPower(intakePower);
        hoods.setPosition(Values.getHoodPos());
        //this is where the automatic intake takes place, if a ball has been intaked, it triggers our main distance sensor and rotates using my custom class
        /*distance = sensors.getIntakeDistance();
        if((distance < 10) && Intaketimer.milliseconds() > 263 && intake){
            //spindexer.sSP(spindexer.getFree(0, spindexer.getPos()),0);
            simpleShoot();
            Intaketimer.reset();


        }*/

    }
    public void shootsetVelocity(double velocity){
        shootMotor1.setVelocity(-velocity);
        shootMotor2.setVelocity(velocity);
    }
    /*public double PIDControl(double reference, double state){
        double error=reference-state;
        double dt = PIDtimer.seconds();
        IntegralSum+=error*dt;
        double derivative=(error-lastError)/dt;
        lastError=error;
        PIDtimer.reset();
        return (error*Kp)+(derivative*Kd)+(IntegralSum*Ki)+(reference*Kf);
    }*/
    public void PIDReset() {
        IntegralSum = 0;
        lastError = 0;
    }
    //this is our regression model to get the correct speed
    public void shootsetPower(double power){
        shootMotor1.setPower(-power);
        shootMotor2.setPower(power);
    }
    public void intakesetPower(double power){
        intakeMotor1.setPower(power);
        intakeMotor2.setPower(-power);
    }
    public void wallPos(double pos){
        wall.setPosition(pos);
    }

    //method to shoot balls and rotate from my class
    public void shoot(){
        //index = spindexer.getFree(1, spindexer.getPos());
        spindexer.sSP(index, 1);
    }
    public void simpleShoot(){
        spindexer.regRot(spindexer.getPos());
    }
    //sets servo positions
    public void setPos(int angle, int offset){
        spindexer.sSP(angle, offset);
    }
    public double getVelocity(){
        return Math.abs(shootMotor1.getVelocity());
    }
    public void stopT(){
        sensors.stopThread();
        Values.stopThread();
    }
    public double turretAngle(){
        return Values.getTurretPos();
    }
}
