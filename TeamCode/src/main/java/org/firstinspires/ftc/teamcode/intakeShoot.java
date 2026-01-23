package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

        private double TargetVelocity = 0;
        private double distance;

        //doubles having to do with PID Tuning (If you dont know PID Tuning look it up, its very useful)
        private double IntegralSum = 0;
        private double lastError = 0;
        public static double Kp=0.0047;
        public static double Ki=0.0004;
        public static double Kd=0;
        public static double Kf=0;

    //ints
    private int shootMode = 1;
    private int index;
    private int intakeMode = 0;
    private int arrayShootIntakeTrack;

    //various timers for delaying stuff (super useful in a lot of scenarios)
    private ElapsedTime PIDtimer = new ElapsedTime();
    private ElapsedTime Intaketimer = new ElapsedTime();

    public intakeShoot(HardwareMap hardwareMap, String intake1, String intake2, String shoot1, String shoot2, String servoName, String servoName2, String distance1, String distance2, String distance3, String distance4, String distance5, String distance6, String distance7, String wallName) {
        //constructor this is where everything is initialized
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, intake1);
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, intake2);
        wall = hardwareMap.get(Servo.class, wallName);
        shootMotor1 = hardwareMap.get(DcMotorEx.class, shoot1);
        shootMotor2 = hardwareMap.get(DcMotorEx.class, shoot2);

        //starting all the timers
        PIDtimer.reset();
        Intaketimer.reset();

        //my custom class takes all of these variables
        spindexer = new servo720Rot(hardwareMap, servoName, servoName2, distance1, distance2, distance3, distance4, distance5, distance6, distance7);

    }
    //most of the times useful to have an update method to update servo positions or motor powers and other stuff
    public void update(double intakePower, int pathstate, Telemetry telemetry, boolean intake){
        //pathstate is from the auto and tells us which stage the robot is in and like if it is done or not
        /*if(pathstate == 16) {
            shootPower = 0;
            intakePower = 0;
        }
        else {
            //using PID and a targetVelocity in order to keep the right motor velocity
            TargetVelocity = getGoodVel(distance);
            shootPower = 1;

        }
        //setting the power of the shooter and intake here
        shootsetPower(shootPower);
        intakesetPower(intakePower);*/
        //this is where the automatic intake takes place, if a ball has been intaked, it triggers our main distance sensor and rotates using my custom class
        if((spindexer.getDisMain() < 10 || spindexer.distanceSensors[0].getDistance(DistanceUnit.CM) < 5) && Intaketimer.milliseconds() > 400 && intake){
            //spindexer.sSP(spindexer.getFree(0, spindexer.getPos()),0);
            spindexer.regRot(spindexer.getPos());
            Intaketimer.reset();


        }
        telemetry.addData ( "good index", index);
        telemetry.addData("disDiss", spindexer.getDisMain());
        telemetry.addData("dis2", spindexer.distanceSensors[0].getDistance(DistanceUnit.CM));
        telemetry.addData("dis3", spindexer.distanceSensors[1].getDistance(DistanceUnit.CM));
        telemetry.addData("dis4", spindexer.distanceSensors[2].getDistance(DistanceUnit.CM));
        telemetry.addData("dis5", spindexer.distanceSensors[3].getDistance(DistanceUnit.CM));
        telemetry.addData("dis6", spindexer.distanceSensors[4].getDistance(DistanceUnit.CM));
        telemetry.addData("dis7", spindexer.distanceSensors[5].getDistance(DistanceUnit.CM));
    }
    public double PIDControl(double reference, double state){
        double error=reference-state;
        double dt = PIDtimer.seconds();
        IntegralSum+=error*dt;
        double derivative=(error-lastError)/dt;
        lastError=error;
        PIDtimer.reset();
        return (error*Kp)+(derivative*Kd)+(IntegralSum*Ki)+(reference*Kf);
    }
    public void PIDReset() {
        IntegralSum = 0;
        lastError = 0;
    }
    //this is our regression model to get the correct speed
    public double getGoodVel(double dis){
        return 0.0000189394*(dis*dis*dis*dis) - 0.00598485*(dis*dis*dis) + 0.70947*(dis*dis) - 34.90476*(dis) + 1655.01299;
    }
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
       index = spindexer.getFree(1, spindexer.getPos());
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
}
