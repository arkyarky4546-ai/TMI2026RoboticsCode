package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
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

//added jimmy's code
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
    double temp = 0.0;
    private int arrayShootIntakeTrack;
    private int shootSequenceStep = 0;

    //various timers for delaying stuff (super useful in a lot of scenarios)
    private ElapsedTime PIDtimer = new ElapsedTime();
    private ElapsedTime Intaketimer = new ElapsedTime();
    private sensCalc1 sensors;
    private shooterThread Values;
    private Servo hoods;
    private Shooter shooter = new Shooter();

    //teleop stuff
    private double intakePower;
    private final double WALL_SHOOT = 0.5;
    private final double WALL_UP = 0.1;
    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean setHoodVelocityTurret;
    final int BLUE = 1;
    final int RED = 2;
    private int mode;


    public intakeShoot(HardwareMap hardwareMap, String intake1, String intake2, String shoot1, String shoot2, String servoName, String servoName2, String wallName, String colorS1, String colorS2, String shooterHood, Follower follower) {
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
        spindexer = new servo720Rot(hardwareMap, servoName, servoName2, colorS1, colorS2);
        spindexer.sSP(0,0);
        //sensors = new sensCalc1(spindexer);
        //sensors.start();
        Values = new shooterThread(shooter, follower, ShooterConstants.GOAL_POSE_BLUE, follower.getHeading());
        Values.start();

        //more teleop stuff
        intakePower = 0.75;
        wallPos(WALL_UP);
        setHoodVelocityTurret = false;


    }
    //most of the times useful to have an update method to update servo positions or motor powers and other stuff
    public void update(double intakePower, int pathstate, boolean intake, Follower follower){
        Values.update(follower, ShooterConstants.GOAL_POSE_BLUE, follower.getHeading());
        if(pathstate == 16) {
            shootPower = 0;
            intakePower = 0;
        }
        else {

            shootsetVelocity(Values.getSpeed());

        }
        //setting the power of the shooter and intake here
        shootsetPower(shootPower);
        intakesetPower(intakePower);

        hoods.setPosition(MathFunctions.clamp(Values.getHoodPos(), 0.0, 1));
        //this is where the automatic intake takes place, if a ball has been intaked, it triggers our main distance sensor and rotates using my custom class
        /*distance = sensors.getIntakeDistance();
        if((distance < 10) && Intaketimer.milliseconds() > 263 && intake){
            //spindexer.sSP(spindexer.getFree(0, spindexer.getPos()),0);
            simpleShoot();
            Intaketimer.reset();


        }*/

    }

    /*Teleop Update*/
    public void update(boolean intakeActive, boolean intakeOut, boolean shootActive, Follower follower) {
        if(!setHoodVelocityTurret){
            Values.update(follower, ShooterConstants.GOAL_POSE_BLUE, follower.getHeading());
            shootsetVelocity(1000);
            hoods.setPosition(MathFunctions.clamp(Values.getHoodPos(), 0.0, 1));
        }

        if (intakeActive) {
            intakesetPower(intakePower);
            wallPos(WALL_UP);
            shootSequenceStep = 0;
        }
        else if (intakeOut) {
            intakesetPower(-intakePower);
            wallPos(WALL_UP);
            shootSequenceStep = 0;
        }
        else if (shootActive) {
            //shootsetVelocity(Values.getSpeed());
            intakesetPower(0.75);
            if (shootSequenceStep == 0) {
                wallPos(WALL_UP);
                shootTimer.reset();  // Start the failsafe stopwatch
                shootSequenceStep = 1;
            }
            if(shootSequenceStep == 1 && shootTimer.milliseconds() > 500){
                spindexer.sSP(0, 0); // Send it to index 0
                shootTimer.reset();  // Start the failsafe stopwatch
               // if (spindexer.isAtTarget())
                shootSequenceStep = 2;
            }

            else if (shootSequenceStep == 2) {
                // Wait until the analog sensor confirms arrival, OR 500ms passes (failsafe)
                if (spindexer.isAtTarget() || shootTimer.milliseconds() > 500) {
                    wallPos(WALL_SHOOT);
                    shootTimer.reset();
                    shootSequenceStep = 3;
                }
            }

            else if (shootSequenceStep == 3) {
                // Wait until the analog sensor confirms arrival, OR 500ms passes (failsafe)
                if (shootTimer.milliseconds() > 500) {
                    spindexer.fastRot(spindexer.getPos());
                    shootTimer.reset();
                    shootSequenceStep = 4;
                }
            }

            else if (shootSequenceStep == 4) {
                if (spindexer.isAtTarget() || shootTimer.milliseconds() > 500) {
                    // shot complete
                    shootSequenceStep = -1;
                    shootTimer.reset();
                }
            }
            else if(shootTimer.milliseconds() > 4000){
                shootSequenceStep = 0;
            }
        }
        else{
            wallPos(WALL_UP);
            intakeMotor1.setPower(0);
            intakeMotor2.setPower(0);
            shootTimer.reset();
            shootSequenceStep = 0;
        }
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
        wall.setPosition(MathFunctions.clamp(pos,0,1));
    }

    public void hoodPos(double pos){
        hoods.setPosition(MathFunctions.clamp(pos, 0, 1));
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
       // sensors.stopThread();
        Values.stopThread();
    }
    public double turretAngle(){
        return Values.getTurretPos();
    }
    public void fastShoot(){
        spindexer.fastRot(spindexer.getPos());
    }
    public double findGreen(){
        if (spindexer.getColors() == 1){
            return spindexer.getPos();
        }
        simpleShoot();
        return 0.0;
    }
    public void colorSort(double Position, int[]Pattern){
        spindexer.sort(Position, Pattern);
    }
    public void setModeBlue(){
        mode = BLUE;
    }

    public void setModeRed(){
        mode = RED;
    }

    public void setShootFar(){
        setHoodVelocityTurret = !setHoodVelocityTurret;
        if(mode == BLUE){
            hoods.setPosition(0.4);
            shootsetVelocity(1500);
        }
        else if(mode == RED){

        }
    }
}
