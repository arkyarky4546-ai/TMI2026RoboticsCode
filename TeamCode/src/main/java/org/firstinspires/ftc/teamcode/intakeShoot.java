package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.revisedTeleOp.SensorThread;
import org.firstinspires.ftc.teamcode.revisedTeleOp.sensCalc;
import org.firstinspires.ftc.teamcode.revisedTeleOp.sensCalc1;
import org.firstinspires.ftc.teamcode.servo720Rot;
//added jimmy's code
public class intakeShoot {

    //DcMotors
    private DcMotorEx intakeMotor1, intakeMotor2, shootMotor1, shootMotor2;

    //my spindex rotator class. look at servo720Rot
    private servo720Rot spindexer;
    private SensorThread sensorReader;

    private Servo wall;
    private Servo ceiling;
    private Servo rail1;
    //doubles
    //variable that controls shooting power
    private double shootPower;
    private double ticksPerRev = 28;
    private double TargetVelocity = 0;
    private double distance;

    // --- PID VARIABLES COMMENTED OUT ---
    // private double IntegralSum = 0;
    // private double lastError = 0;
    // public static double Kp=0.0121;
    // public static double Ki=0.00014;
    // public static double Kd=0.0000;
    // public static double Kf=.0000;

    //ints
    private int shootMode = 1;
    int index;
    private int intakeMode = 0;
    double temp = 0.0;
    private int arrayShootIntakeTrack;
    private int shootSequenceStep = 0;
    private int debugSequenceStep = 0;

    //various timers for delaying stuff (super useful in a lot of scenarios)
    private ElapsedTime PIDtimer = new ElapsedTime();
    private ElapsedTime Intaketimer = new ElapsedTime();
    private sensCalc1 sensors;
    private shooterThread Values;
    private Servo hoods;
    private Shooter shooter = new Shooter();
    private double recoil = .025;
    private boolean shootingBall = false;

    //teleop stuff
    private double intakePower;
    private final double WALL_SHOOT = 0.5;
    private final double WALL_UP = 0.36;
    private final double WALL_UP1 = 0.2;
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime debugTimer = new ElapsedTime();
    private boolean setHoodVelocityTurret;
    final int BLUE = 1;
    final int RED = 2;
    private int mode;
    private boolean shootAc = true;
    private DistanceSensor disBL;
    private DistanceSensor disST;
    private ElapsedTime reverseTimer = new ElapsedTime();
    private ElapsedTime shooting = new ElapsedTime();
    private boolean wasIntaking = false;
    private boolean isReversing = false;
    private disAndColor colorShoot;
    private double currentPos = 0.0;
    private double railDOWN = .512;
    private double railUP= .38;
    private double railUP1= .644;
    private double ceilingDOWN = 0.69;
    private double ceilingUP = 0.55;
    private double lastError = 0;
    public static double kV = 0.00045;

    public static double kS = .155
            ,kP = 0.012;

    private double currentValue = 0.0;
    private boolean shootReset = false;
    ElapsedTime recoilTimer=new ElapsedTime();

    public boolean nIsTrue = false;
    ElapsedTime timer=new ElapsedTime();
    ElapsedTime intakeOutTimer = new ElapsedTime();
    double lasterror=0;
    double Integralsum=0;
    double shooterPower = .5;
    private double cpm = 28;

    boolean outake = false;
    boolean far = false;
    Servo rail;
    public static double SPINOFFSET = 50;
    public static double SHOOTOFFSET = 0;
    public static double SHOOTOFFSET1 = 0;
    public static double HOODOFFSET = .00;
    private boolean sortWall = false;
    private ElapsedTime oscillationTimer = new ElapsedTime();

    public intakeShoot(HardwareMap hardwareMap, String intake1, String intake2, String shoot1, String shoot2, String servoName, String servoName2, String wallName, String colorS1, String colorS2, String shooterHood, Follower follower) {
        //constructor this is where everything is initialized
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, intake1);
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, intake2);
        wall = hardwareMap.get(Servo.class, wallName);
        shootMotor1 = hardwareMap.get(DcMotorEx.class, shoot1);
        shootMotor2 = hardwareMap.get(DcMotorEx.class, shoot2);
        shootMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hoods = hardwareMap.get(Servo.class, shooterHood);
        disBL = hardwareMap.get(DistanceSensor.class, "disBL");
        disST = hardwareMap.get(DistanceSensor.class, "disST");
        rail = hardwareMap.get(Servo.class, "rail");
        rail1 = hardwareMap.get(Servo.class, "rail1");
        ceiling = hardwareMap.get(Servo.class, "ceiling");

        //starting all the timers
        PIDtimer.reset();
        Intaketimer.reset();
        sensorReader = new SensorThread(disBL, disST);
        sensorReader.start();

        //my custom class takes all of these variables
        spindexer = new servo720Rot(hardwareMap, servoName, servoName2, colorS1, colorS2);
        spindexer.sSP(0,0);
        //sensors = new sensCalc1(spindexer);
        //sensors.start();
        Values = new shooterThread(shooter, follower, ShooterConstants.GOAL_POSE_BLUE, follower.getHeading());
        Values.start();
        colorShoot = new disAndColor(spindexer);
        colorShoot.start();


        //more teleop stuff
        intakePower = 1;
        wallPos(WALL_UP);
        setHoodVelocityTurret = false;
        ceiling.setPosition(ceilingUP);
        rail.setPosition(railUP);
        rail1.setPosition(railUP1);

    }
    public double PIDControl(double targ, double state){
        double error=targ-state;
        double feedForward = kV*targ + kS;
        double fb = error * kP;

        return feedForward +fb;
    }
    //most of the times useful to have an update method to update servo positions or motor powers and other stuff

    public void update(boolean intakeActive, boolean patternCycle, boolean gateIntakeOut, boolean shootActive, boolean debugActive, Follower follower, Telemetry telemetry, boolean sort, boolean intakeout1, boolean gatePause) {
        if(mode == BLUE) {
            Values.update(follower, ShooterConstants.GOAL_POSE_BLUE, follower.getHeading());
            SHOOTOFFSET = 25;
            SHOOTOFFSET1 = 25;
        }
        else{
            Values.update(follower, ShooterConstants.GOAL_POSE_RED, follower.getHeading());
            SHOOTOFFSET = 50;
            SHOOTOFFSET1 = 25;
        }

        double current = Math.abs(getVelocity());

        try{
            if(follower.getPose().getX() < 30) {
                far = true;
                shooterPower = PIDControl(Values.getSpeed() + SHOOTOFFSET, current);
                if(!shootingBall) {
                    hoods.setPosition(Range.clip(Values.getHoodPos() + HOODOFFSET, 0.0, .57));
                }
            }
            else{
                far = false;
                shooterPower = PIDControl(Values.getSpeed() + SHOOTOFFSET1, current);
                if(!shootingBall) {
                    hoods.setPosition(Range.clip(Values.getHoodPos(), 0.0, .57));
                }

            }

        }
        catch (NullPointerException e){
            ;nIsTrue = true;
        }
        telemetry.addData("Theospeed", Values.getSpeed());
        telemetry.addData("Acspeed", current);
        telemetry.addData("nIsTrue", nIsTrue);
        telemetry.addData("turretAngle", Values.getTurretPos());
        telemetry.addData("speed", Values.getSpeed());
        telemetry.addData("hoodAngle", Values.getHoodPos());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        shootsetPower(shooterPower);
       // shootsetVelocity(1000);
        colorShoot.upColor(spindexer.getPos());
        if (intakeActive) {
            //spindexer.sSP(0,0);
            intakesetPower(intakePower);
            //wallPos(WALL_UP);
            shootSequenceStep = 0;
            ceiling.setPosition(ceilingUP);
        }
        else if (gateIntakeOut) {
            intakesetPower(-intakePower);
            wallPos(WALL_UP);
            shootSequenceStep = 0;
            ceiling.setPosition(ceilingUP);
            if(gateIntakeOut){
                rail.setPosition(railUP);
                rail1.setPosition(railUP1);
            }
        }
        else if(patternCycle){
            intakesetPower(-intakePower);
            rail.setPosition(railDOWN);
            rail1.setPosition(railDOWN);
        }
        else if (intakeout1) {
            intakesetPower(-intakePower);
            //wallPos(WALL_UP);
            //shootSequenceStep = 0;
            //ceiling.setPosition(ceilingUP);
            //if(gateIntakeOut){
            //    rail.setPosition(railUP);
              //  rail1.setPosition(railUP1);
            //}
        }

        else if (debugActive) {
           // wallPos(WALL_UP);
            if(sensorReader.hasBallBL() && sensorReader.hasBallST()){
                spindexer.sSP(0, 0);
            }else if (sensorReader.hasBallBL()) {
                spindexer.sSP(1, 0);
            } else if (sensorReader.hasBallST()) {
                spindexer.sSP(1, 0);
            } else {
                spindexer.sSP(0, 0);
            }
        }
        else if(shootActive){

            shootingBall = true;
            if(shootAc){
                shootAc = false;
                shooting.reset();
                currentPos = spindexer.getPos();

            }
            rail.setPosition(railDOWN);
            rail1.setPosition(railDOWN);
            intakesetPower(1);
            wallPos(WALL_SHOOT);
            ceiling.setPosition(ceilingDOWN);
            if(!far) {
                if (recoilTimer.milliseconds() > 40 && currentValue - .13 < hoods.getPosition() && shootReset) {
                    hoods.setPosition(hoods.getPosition() - recoil);
                    recoilTimer.reset();
                }

            }
            else {
                if (recoilTimer.milliseconds() > 30 && currentValue - .13 < hoods.getPosition() && shootReset) {
                    hoods.setPosition(hoods.getPosition() - recoil);
                    recoilTimer.reset();
                }
            }
            if(!far){
                if(shooting.milliseconds() > 200){
                    if(!shootReset){
                        shootReset = true;
                        recoilTimer.reset();
                        currentValue = Values.getHoodPos();
                    }
                    fastShootREAL(currentPos);
                    rail.setPosition(railDOWN);
                    rail1.setPosition(railDOWN);
                    //shooting.reset();
                }


            }
            else{
                if(shooting.milliseconds() > 500){
                    shooting.reset();
                    if(!shootReset){
                        shootReset = true;
                        recoilTimer.reset();
                        currentValue = Values.getHoodPos();
                    }
                    rail.setPosition(railDOWN);
                    rail1.setPosition(railDOWN);
                    simpleShoot();
                }


            }
        }
        /*else if (shootActive) {
            shootsetVelocity(Values.getSpeed());
            intakesetPower(1);

            //  Wake up / Loop restart (Wall goes UP instantly)
            if (shootSequenceStep == 0 || shootSequenceStep == 12) {
                //spindexer.sSPT();
                wallPos(WALL_UP);
                shootTimer.reset();
                shootSequenceStep = 1;
            }

            else if (shootSequenceStep == 1) {
                if (shootTimer.milliseconds() > 400) {
                    spindexer.sSPT();
                    shootTimer.reset();
                    //wallPos(WALL_SHOOT);
                    shootSequenceStep = 2;
                }
            }

            else if (shootSequenceStep == 2) {
                if (spindexer.isAtTarget() || shootTimer.milliseconds() > 1500) {
                    shootTimer.reset();
                    shootSequenceStep = 3;
                }
            }

            else if (shootSequenceStep == 3) {
                if (shootTimer.milliseconds() > 150) {
                    wallPos(WALL_SHOOT);
                    shootTimer.reset();
                    shootSequenceStep = 4;
                }
            }

            //fire
            else if (shootSequenceStep == 4) {
                if (shootTimer.milliseconds() > 500) {
                    spindexer.fastRot(spindexer.getPos());
                    shootTimer.reset();
                    shootSequenceStep = 5;
                }
            }

            //detect jamming
            else if (shootSequenceStep == 5) {
                if (shootTimer.milliseconds() > 1500) {
                    shootSequenceStep = 0; // Loop back to the top
                }
            }
        }*/
        else {
            // IDLE STATE (Driver let go of all buttons)
            intakeMotor1.setPower(0);
            intakeMotor2.setPower(0);
            shootAc = true;
            shootReset = false;
            shootingBall = false;


            ceiling.setPosition(ceilingUP);
            if (sort){
                if(!sortWall) {
                    sortWall = true;
                    spindexer.sSP(0, 1);
                }
                wallPos(WALL_UP1);
                rail.setPosition(railDOWN);
                rail1.setPosition(railDOWN);
            }
            else if (!sort) {
                spindexer.sSP(0, 0);
                sortWall = false;
                wallPos(WALL_UP);
                rail.setPosition(railUP);
                rail1.setPosition(railUP1);
            }
            // IDLE CLEANUP SEQUENCE
            /*if (shootSequenceStep < 10) {
                wallPos(WALL_SHOOT);
                shootTimer.reset();
                shootSequenceStep = 10;
            }
            else if (shootSequenceStep == 10) {
                if (shootTimer.milliseconds() > 500) {
                    spindexer.sSP(0, 0);
                    shootSequenceStep = 12;
                }
            }
            else if (shootSequenceStep == 12) {
                // safe zone
            }*/

        }
        spindexer.update();
    }
    public void update1(boolean intakeActive, boolean intakeOut, boolean gateIntakeOut, boolean shootActive, boolean debugActive, Follower follower, Telemetry telemetry,boolean auto, boolean sort) {
        if(mode == BLUE) {
            Values.update(follower, ShooterConstants.GOAL_POSE_BLUE, follower.getHeading());
            SHOOTOFFSET = 25;
        }
        else{
            Values.update(follower, ShooterConstants.GOAL_POSE_RED, follower.getHeading());
            SHOOTOFFSET = 50;
        }

        double current = Math.abs(getVelocity());

        try{
            if(follower.getPose().getX() < 30) {
                far = true;
                shooterPower = PIDControl(Values.getSpeed() + 50, current);
                if(!shootingBall) {
                    hoods.setPosition(Range.clip(Values.getHoodPos() - HOODOFFSET, 0.0, .57));
                }
            }
            else{
                far = false;
                shooterPower = PIDControl(Values.getSpeed() + 20 + SHOOTOFFSET, current);
                if(!shootingBall) {
                    hoods.setPosition(Range.clip(Values.getHoodPos(), 0.0, .57));
                }

            }

        }
        catch (NullPointerException e){
            ;nIsTrue = true;
        }
        telemetry.addData("Theospeed", Values.getSpeed());
        telemetry.addData("Acspeed", current);
        telemetry.addData("nIsTrue", nIsTrue);
        telemetry.addData("turretAngle", Values.getTurretPos());
        telemetry.addData("speed", Values.getSpeed());
        telemetry.addData("hoodAngle", Values.getHoodPos());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        shootsetPower(shooterPower);
        if(!auto) {
            intakesetPower(intakePower);
        }
        if (auto){
            intakesetPower(-intakePower);
            rail.setPosition(railDOWN);
            rail1.setPosition(railDOWN);
        }
        // shootsetVelocity(1000);
        colorShoot.upColor(spindexer.getPos());
        if (intakeActive) {
            //spindexer.sSP(0,0);
            intakesetPower(intakePower);
            //wallPos(WALL_UP);
            shootSequenceStep = 0;
            ceiling.setPosition(ceilingUP);
        }
        else if (intakeOut || gateIntakeOut) {
            intakesetPower(-intakePower);
            wallPos(WALL_UP);
            shootSequenceStep = 0;
            ceiling.setPosition(ceilingUP);
            if(gateIntakeOut){
                rail.setPosition(railUP);
                rail1.setPosition(railUP1);
            }
        }
        if (auto){
            intakesetPower(-intakePower);
        }

        else if (debugActive) {
            // wallPos(WALL_UP);
            if(sensorReader.hasBallBL() && sensorReader.hasBallST()){
                spindexer.sSP(0, 0);
            }else if (sensorReader.hasBallBL()) {
                spindexer.sSP(1, 0);
            } else if (sensorReader.hasBallST()) {
                spindexer.sSP(1, 0);
            } else {
                spindexer.sSP(0, 0);
            }
        }
        else if(shootActive){

            shootingBall = true;
            if(shootAc){
                shootAc = false;
                shooting.reset();
                currentPos = spindexer.getPos();

            }
            rail.setPosition(railDOWN);
            rail1.setPosition(railDOWN);
            intakesetPower(1);
            wallPos(WALL_SHOOT);
            ceiling.setPosition(ceilingDOWN);
            if(!far) {
                if (recoilTimer.milliseconds() > 37 && currentValue - .15 < hoods.getPosition() && shootReset) {
                    hoods.setPosition(hoods.getPosition() - recoil);
                    recoilTimer.reset();
                }

            }
            else {
                if (recoilTimer.milliseconds() > 30 && currentValue - .15 < hoods.getPosition() && shootReset) {
                    hoods.setPosition(hoods.getPosition() - recoil);
                    recoilTimer.reset();
                }
            }
            if(!far){
                if(shooting.milliseconds() > 200){
                    if(!shootReset){
                        shootReset = true;
                        recoilTimer.reset();
                        currentValue = Values.getHoodPos();
                    }
                    fastShootREAL(currentPos);
                    rail.setPosition(railDOWN);
                    rail1.setPosition(railDOWN);
                    //shooting.reset();
                }


            }
            else{
                if(shooting.milliseconds() > 600){
                    shooting.reset();
                    if(!shootReset){
                        shootReset = true;
                        recoilTimer.reset();
                        currentValue = Values.getHoodPos();
                    }
                    rail.setPosition(railDOWN);
                    rail1.setPosition(railDOWN);
                    simpleShoot();
                }


            }
        }
        /*else if (shootActive) {
            shootsetVelocity(Values.getSpeed());
            intakesetPower(1);

            //  Wake up / Loop restart (Wall goes UP instantly)
            if (shootSequenceStep == 0 || shootSequenceStep == 12) {
                //spindexer.sSPT();
                wallPos(WALL_UP);
                shootTimer.reset();
                shootSequenceStep = 1;
            }

            else if (shootSequenceStep == 1) {
                if (shootTimer.milliseconds() > 400) {
                    spindexer.sSPT();
                    shootTimer.reset();
                    //wallPos(WALL_SHOOT);
                    shootSequenceStep = 2;
                }
            }

            else if (shootSequenceStep == 2) {
                if (spindexer.isAtTarget() || shootTimer.milliseconds() > 1500) {
                    shootTimer.reset();
                    shootSequenceStep = 3;
                }
            }

            else if (shootSequenceStep == 3) {
                if (shootTimer.milliseconds() > 150) {
                    wallPos(WALL_SHOOT);
                    shootTimer.reset();
                    shootSequenceStep = 4;
                }
            }

            //fire
            else if (shootSequenceStep == 4) {
                if (shootTimer.milliseconds() > 500) {
                    spindexer.fastRot(spindexer.getPos());
                    shootTimer.reset();
                    shootSequenceStep = 5;
                }
            }

            //detect jamming
            else if (shootSequenceStep == 5) {
                if (shootTimer.milliseconds() > 1500) {
                    shootSequenceStep = 0; // Loop back to the top
                }
            }
        }*/
        else {
            // IDLE STATE (Driver let go of all buttons)
            //intakeMotor1.setPower(0);
            //intakeMotor2.setPower(0);
            shootAc = true;
            shootReset = false;
            shootingBall = false;


            ceiling.setPosition(ceilingUP);
            if (sort){
                if(!sortWall) {
                    sortWall = true;
                    spindexer.sSP(0, 1);
                }
                wallPos(WALL_UP1);
                rail.setPosition(railDOWN);
                rail1.setPosition(railDOWN);
            }
            else if (!sort) {
                spindexer.sSP(0, 0);
                sortWall = false;
                wallPos(WALL_UP);
                rail.setPosition(railUP);
                rail1.setPosition(railUP1);
            }
            // IDLE CLEANUP SEQUENCE
            /*if (shootSequenceStep < 10) {
                wallPos(WALL_SHOOT);
                shootTimer.reset();
                shootSequenceStep = 10;
            }
            else if (shootSequenceStep == 10) {
                if (shootTimer.milliseconds() > 500) {
                    spindexer.sSP(0, 0);
                    shootSequenceStep = 12;
                }
            }
            else if (shootSequenceStep == 12) {
                // safe zone
            }*/

        }
        spindexer.update();
    }
    public void update(boolean intakeActive, boolean intakeOut, boolean shootActive, boolean debugActive, Follower follower, Telemetry telemetry, boolean auto, boolean far) {
        if(mode == BLUE) {
            Values.update(follower, ShooterConstants.GOAL_POSE_BLUE, follower.getHeading());
        }
        else{
            Values.update(follower, ShooterConstants.GOAL_POSE_BLUE, follower.getHeading());
        }
        double current = Math.abs(getVelocity());

        shooterPower = PIDControl(Values.getSpeed(), current);
        telemetry.addData("turretAngle", Values.getTurretPos());
        telemetry.addData("speed", Values.getSpeed());
        telemetry.addData("hoodAngle", Values.getHoodPos());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        shootsetPower(shooterPower);
        // shootsetVelocity(1000);
        hoods.setPosition(Range.clip(Values.getHoodPos() +.06, 0.0, 1));
        colorShoot.upColor(spindexer.getPos());
        if (intakeActive) {
            spindexer.sSP(0,0);
            intakesetPower(intakePower);
            wallPos(WALL_UP);
            shootSequenceStep = 0;
            ceiling.setPosition(ceilingUP);
        }
        else if (intakeOut) {
            intakesetPower(-intakePower);
            wallPos(WALL_UP);
            shootSequenceStep = 0;
            ceiling.setPosition(ceilingUP);
        }

        else if (debugActive) {
            // wallPos(WALL_UP);
            if(sensorReader.hasBallBL() && sensorReader.hasBallST()){
                spindexer.sSP(0, 0);
            }else if (sensorReader.hasBallBL()) {
                spindexer.sSP(1, 0);
            } else if (sensorReader.hasBallST()) {
                spindexer.sSP(1, 0);
            } else {
                spindexer.sSP(0, 0);
            }
        }
        else if(shootActive){
            if(shootAc){
                shootAc = false;
                shooting.reset();
                currentPos = spindexer.getPos();

            }
            rail.setPosition(railDOWN);
            rail1.setPosition(railDOWN);
            ceiling.setPosition(ceilingDOWN);
            intakesetPower(1);
            wallPos(WALL_SHOOT);
            if(shooting.milliseconds() > 300){
                simpleShoot();
                shooting.reset();
            }

        }
        else if (auto){
            intakesetPower(-1);
            //intakesetPower(-.5)
        }
        /*else if (shootActive) {
            shootsetVelocity(Values.getSpeed());
            intakesetPower(1);

            //  Wake up / Loop restart (Wall goes UP instantly)
            if (shootSequenceStep == 0 || shootSequenceStep == 12) {
                //spindexer.sSPT();
                wallPos(WALL_UP);
                shootTimer.reset();
                shootSequenceStep = 1;
            }

            else if (shootSequenceStep == 1) {
                if (shootTimer.milliseconds() > 400) {
                    spindexer.sSPT();
                    shootTimer.reset();
                    //wallPos(WALL_SHOOT);
                    shootSequenceStep = 2;
                }
            }

            else if (shootSequenceStep == 2) {
                if (spindexer.isAtTarget() || shootTimer.milliseconds() > 1500) {
                    shootTimer.reset();
                    shootSequenceStep = 3;
                }
            }

            else if (shootSequenceStep == 3) {
                if (shootTimer.milliseconds() > 150) {
                    wallPos(WALL_SHOOT);
                    shootTimer.reset();
                    shootSequenceStep = 4;
                }
            }

            //fire
            else if (shootSequenceStep == 4) {
                if (shootTimer.milliseconds() > 500) {
                    spindexer.fastRot(spindexer.getPos());
                    shootTimer.reset();
                    shootSequenceStep = 5;
                }
            }

            //detect jamming
            else if (shootSequenceStep == 5) {
                if (shootTimer.milliseconds() > 1500) {
                    shootSequenceStep = 0; // Loop back to the top
                }
            }
        }*/

        else {
            // IDLE STATE (Driver let go of all buttons)
            //intakeMotor1.setPower(0);
            //intakeMotor2.setPower(0);
            outake = false;
            shootAc = true;
            rail.setPosition(railUP);
            rail1.setPosition(railUP);
            wallPos(WALL_UP);
            ceiling.setPosition(ceilingUP);
            // IDLE CLEANUP SEQUENCE
            /*if (shootSequenceStep < 10) {
                wallPos(WALL_SHOOT);
                shootTimer.reset();
                shootSequenceStep = 10;
            }
            else if (shootSequenceStep == 10) {
                if (shootTimer.milliseconds() > 500) {
                    spindexer.sSP(0, 0);
                    shootSequenceStep = 12;
                }
            }
            else if (shootSequenceStep == 12) {
                // safe zone
            }*/
            if(oscillationTimer.milliseconds() > 500){
                spindexer.sSP(0,1);
                oscillationTimer.reset();
            }
            else{
                spindexer.sSP(0,0);
            }
        }
        spindexer.update();
    }
    public void shootsetVelocity(double velocity){
        shootMotor1.setVelocity(-velocity);
        shootMotor2.setVelocity(velocity);
    }

    // --- PID CONTROL METHODS COMMENTED OUT ---
    /*public double PIDControl(double reference, double state){
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
    }*/

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
        wall.setPosition(Range.clip(pos,0,1));
    }

    public void hoodPos(double pos){
        hoods.setPosition(Range.clip(pos, 0, 1));
    }

    //method to shoot balls and rotate from my class
    public void shoot(){
        //index = spindexer.getFree(1, spindexer.getPos());
        spindexer.sSP(index, 1);
    }
    public void simpleShoot(){
        spindexer.regRot1(spindexer.getPos());
    }
    public void simpleShoot1(){
        spindexer.regRot2(spindexer.getPos());
    }
    //sets servo positions
    public void setPos(int angle, int offset){
        spindexer.sSP(angle, offset);
    }
    public double getVelocity(){
        return Math.abs(shootMotor2.getVelocity());
    }
    public void slow(int index){
        spindexer.sSSlowP(index);
    }
    public void stopT(){
        // sensors.stopThread();
        Values.stopThread();
        colorShoot.stopThread();
        if (sensorReader != null) {
            sensorReader.stopThread();
        }
    }
    public double turretAngle(){
        return Values.getTurretPos();
    }
    public void fastShoot(){
        spindexer.fastRot(spindexer.getPos());
    }
    public void fastShootREAL(double pos){
        spindexer.fastRot(pos);
    }
    public double findGreen(){
        if (spindexer.getColors() == 1){
            return spindexer.getPos();
        }
        simpleShoot1();
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
    public double getGreen(){
        return colorShoot.foundGreen();
    }
    //public double flywheel(){

    //}

}