package org.firstinspires.ftc.teamcode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutoAimTurret;
import org.firstinspires.ftc.teamcode.AxonRotator;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@Configurable
@TeleOp
public class TeleopGoal3Blue extends OpMode {
    private NormalizedColorSensor coloora;
    private Follower follower;
    // int spoondox;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    Servo turretRight; //launchservo
    Servo turretLeft; //launchservo
    CRServo artifactSpinner;
    Servo artifactPush;
    DcMotorEx shoot1;
    DcMotorEx shoot2;
    DcMotor intakeMotor;
    DcMotor intakeMotor1;
    boolean index12 = false;
    Servo intakeGate;
    Servo shooterHood;
    AxonRotator smart1;
    boolean index1 = false;
    CRServo smart;
    double distance;
    double shooterPower;
    boolean reverse = true;
    double kickZero = 0.85;
    double kickUp = 0.75;
    public static double TURN_Constant = 0.005;
    public boolean aiming;
    double servoPos;
    private Limelight3A limelight3A;
    private double timeBegan;
    private boolean pressShooter;
    double Integralsum = 0;
    public static double Kp=0.0047;
    public static double Ki=0.0004;
    public static double Kd=0;
    public static double Kf=0;
    double TargetVelocity = 1400;
    int green = 1;
    int purple = 2;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime timer12 = new ElapsedTime();
    ElapsedTime timer123 = new ElapsedTime();
    ElapsedTime timer1234 = new ElapsedTime();
    double lasterror = 0;
    double limelightPause;
    int index = 0;
    int sixty = 60;
    int Nsixty = -60;
    int onetwo = 120;
    int Nonetwo = -120;
    int actual = 120;
    Servo wally;
    int coolor;
    float time = System.currentTimeMillis();
    double lastTimeFrame = 0;
    double currTimeFrame = 0;
    int indexForSpindex = 0;
    boolean gateShoot = false;
    double turTurn = .5;
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
    int state = 0;
    int acstate = 0;
    int spoon = 0;
    boolean noLime = false;
    private AutoAimTurret turret;
    private static final double TARGET_X = 72.0; // Field center or specific goal X
    private static final double TARGET_Y = 72.0; // Field center or specific goal Y
    int isBLind = -1;
    @Override
    public void init() {
        //limelight
        dis = hardwareMap.get(DistanceSensor.class, "disDiss");
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(6);

        //pedro pathing stuff
        startingPose = new Pose(80, -60, Math.toRadians(0));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //intake
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");
        intakeGate = hardwareMap.get(Servo.class, "intakeGate");
        intakeGate.setPosition(0);
        //color sensor
        coloora = hardwareMap.get(NormalizedColorSensor.class, "coloora");
        //artifact storage
        // artifactSpinner = hardwareMap.get(CRServo.class, "spindexRoter");
        artifactPush = hardwareMap.get(Servo.class, "push");
        slave = hardwareMap.get(CRServo.class, "slave");
        //shooter
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turretRight.setPosition(turTurn);
        turretLeft.setPosition(turTurn);
        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        wally = hardwareMap.get(Servo.class, "wally");
        // smart = hardwareMap.get(CRServo.class, "spindexRoter");
        smart1 = new AxonRotator(hardwareMap,"spindexRoter", "slave", "smartTrack", reverse);
        //smart2 = new AxonRotator(hardwareMap,"slave", "smartTrack", !reverse);
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterHood = hardwareMap.get(Servo.class, "shooterHood");
        turret = new AutoAimTurret(hardwareMap, "turretLeft", "turretRight");
        turret.setTargetCoordinates(TARGET_X, TARGET_Y);
        shooterHood.setPosition(0.4);
        aiming = true;
        artifactPush.setPosition(kickZero);
        shooterPower = 1;
        //smart1.startRotate120(right);
        timer12.reset();
        timer123.reset();
        timer1.reset();
        indexForSpindex = 0;
        actPattern =0;



        //pedro
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(65, -48, 0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();

        limelightTimer = new Timer();
        limelightTimer.resetTimer();
        turretToLeft = false;
        turretToRight = false;
    }
    public Pose altitudeExist(Pose currentPose){
        double value = Math.abs(Math.abs(currentPose.getX()) + Math.abs(currentPose.getY()))/2;
        return new Pose(value, -(144-value), Math.toRadians(47));
    }
    public boolean scOOON (){
        if (!isScanning){
            isScanning = true;
            numgreen = 0;
            numpurp = 0;
            currentScanPos = 0;
            //scanTimer.reset();
            return false;
        }
        //if (scanTimer.milliseconds()>50){
        //spindexColors[currentScanPos] = getColors();
        currentScanPos++;
        if(currentScanPos >=3){
            isScanning = false;
            return true;
        }
        smart1.startRotate(actual);
        //smart2.startRotate120(actual);
        //}
        return false;
    }
    public void rotateSpindex(){ //what the helly, thread.sleep sucks
        smart1.startRotate(actual);
        //smart2.startRotate120(actual);
    }
    public double PIDControl(double reference, double state){
        double error=reference-state;
        double dt = timer.seconds();
        Integralsum+=error*dt;
        double derivative=(error-lasterror)/dt;
        lasterror=error;
        timer.reset();

        double output=(error*Kp)+(derivative*Kd)+(Integralsum*Ki)+(reference*Kf);
        return output;
    }
    //public double slopeTopCalc(double x)

    public double getGoodVel(double dis){
        return -217*(dis*dis*dis) + 875.6403*(dis*dis) -1196.11498*(dis) + 1830.8098;
    }
    public double getGoodHood(double dis){
        return- 0.145141*(dis*dis)+0.397149 * (dis) +0.30438;
    }
    public int shootOneBall(){ //wasn't letting me use thread.sleep for some reason so it told me to add this
        wally.setPosition(0.3);
        Integralsum = 0;
        lasterror = 0;
        actual = onetwo;
        TargetVelocity = getGoodVel(distance);
        //TargetVelocity = 1400;
        shooterHood.setPosition(.4);
        double current = shoot2.getVelocity();
        shooterPower = PIDControl(TargetVelocity+100, current);
        shoot1.setPower(-shooterPower);
        shoot2.setPower(shooterPower);
        intakeMotor.setPower(1);
        intakeMotor1.setPower(-1);
        //artifactSpinner.setPower(-1);
                /*if (!pressShooter) {
                    pressShooter = true;
                    index1 = false;
                    timeBegan = System.currentTimeMillis();
                }*/
        //if (timeBegan + 500 < System.currentTimeMillis()) {

        if(shoot2.getVelocity()>(.92*TargetVelocity)){
            index1 = false;
            wally.setPosition(0);
            artifactPush.setPosition(kickUp);

            //artifactPush.setPosition(kickZero);
            timer1234.reset();
            return 1;

            //gateShoot = false;
        }
        else {
            wally.setPosition(.3);
            return 0;
            //artifactPush.setPosition(kickZero);
            //gateShoot = true;
        }
        //}
        //turretLeft.setPosition();
        //turretRight.setPosition();
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();

        limelight3A.start();
    }
    /*public int getColors(){
        telemetry.addData("Light Detected", ((OpticalDistanceSensor) coloora).getLightDetected());
        NormalizedRGBA colors = coloora.getNormalizedColors();
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
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);
        telemetry.addData("pos0", spindexColors[0]);
        telemetry.addData("pos1", spindexColors[1]);
        telemetry.addData("pos2", spindexColors[2]);
        return color;
    }*/
    @Override
    public void loop() {
        double doos = dis.getDistance(DistanceUnit.CM);
        //coolor = getColors();
        smart1.update(telemetry);
        //slave.setPower(-smart1.getPower());
        // smart2.update(telemetry);
        //timer1.milliseconds();
        //telemetry.addData("time", timer1.milliseconds());
        //timer1.reset();
        //Call this once per loop
        follower.update();
        telemetryM.update();
        LLResult result = limelight3A.getLatestResult();
        //if(aiming){
        if (index == 0) {
            limelightPause = System.currentTimeMillis();
            time = System.currentTimeMillis();
            timer123.reset();
            timer12.reset();
            index = 67; // I am an adult
        }
        if (!noLime && isBLind<0) {
            boolean wentIn = false;
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
                distance = result.getTa();
                for (LLResultTypes.FiducialResult tag : results) {
                    if (tag.getFiducialId() == 23) {
                        actPattern = 1;
                    } else if (tag.getFiducialId() == 22) {
                        actPattern = 2;
                    } else if (tag.getFiducialId() == 21) {
                        actPattern = 3;
                    } else if (tag.getFiducialId() == 20) {
                        wentIn = true;
                        if (limelightTimer.getElapsedTime() > 1) {
                            double tx = tag.getTargetXDegrees();
                            double toBeSetPos = turretLeft.getPosition();
                            if (tx < -5) {
                                toBeSetPos -= 0.01;
                                turretToRight = false;
                                turretToLeft = false;
                            } else if (tx > 5) {
                                toBeSetPos += 0.01;
                                turretToRight = false;
                                turretToLeft = false;
                            }
                            if (toBeSetPos < 0.26) {
                                turretToRight = true;
                                turretToLeft = false;
                                turningLeft = false;
                                turningRight = false;
                                toBeSetPos = 0.25;
                            } else if (toBeSetPos > 0.74) {
                                turretToLeft = true;
                                turretToRight = false;
                                turningLeft = false;
                                turningRight = false;
                                toBeSetPos = 0.75;
                            }
                            if (turretToLeft) {
                                toBeSetPos = turretLeft.getPosition() - 0.01;
                                if (toBeSetPos < 0.26) {
                                    turretToLeft = false;
                                }
                            }
                            if (turretToRight) {
                                toBeSetPos = turretLeft.getPosition() + 0.01;
                                if (toBeSetPos > 0.74) {
                                    turretToRight = false;
                                }
                            }
                            turretLeft.setPosition(toBeSetPos);
                            turretRight.setPosition(toBeSetPos);
                            limelightTimer.resetTimer();
                        }
                    }


                }
            }
            if ((limelightTimer.getElapsedTime() > 1) && !wentIn) {
                double toBeSetPos = turretLeft.getPosition();
                if (turningRight) {
                    toBeSetPos -= 0.02;
                    turretToRight = false;
                    turretToLeft = false;
                } else if (turningLeft) {
                    toBeSetPos += 0.02;
                    turretToRight = false;
                    turretToLeft = false;
                }
                if (toBeSetPos < 0.26) {
                    turretToRight = true;
                    turretToLeft = false;
                    turningLeft = false;
                    turningRight = false;
                    toBeSetPos = 0.25;
                } else if (toBeSetPos > 0.74) {
                    turretToLeft = true;
                    turretToRight = false;
                    turningLeft = false;
                    turningRight = false;
                    toBeSetPos = 0.75;
                }
                if (turretToLeft) {
                    toBeSetPos = turretLeft.getPosition() - 0.02;
                    if (toBeSetPos < 0.26) {
                        turretToLeft = false;
                    }
                }
                if (turretToRight) {
                    toBeSetPos = turretLeft.getPosition() + 0.02;
                    if (toBeSetPos > 0.74) {
                        turretToRight = false;
                    }
                }
                turretLeft.setPosition(toBeSetPos);
                turretRight.setPosition(toBeSetPos);
                limelightTimer.resetTimer();
            }
            if ((gamepad1.right_stick_x < -0.1) && !turretToLeft && !turretToRight) {
                turningLeft = true;
                turningRight = false;
            }
            if ((gamepad1.right_stick_x > 0.1) && !turretToLeft && !turretToRight) {
                turningRight = true;
                turningLeft = false;
            }
        }
        if(gamepad2.a){
            turretLeft.setPosition(0.55);
            turretRight.setPosition(0.55);
        }
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.xWasPressed()) {
            Pose targetPose = altitudeExist(follower.getPose());
            /*telemetry.addData("x: ", follower.getPose().getX());
            telemetry.addData("y: ", follower.getPose().getY());
            telemetry.addData("going to x: ", targetPose.getX());
            telemetry.addData("going to y: ", targetPose.getY());*/
            PathChain twoPointsOrSmthn = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), targetPose))
                    .setLinearHeadingInterpolation(follower.getHeading(), targetPose.getHeading())
                    .build();
            follower.followPath(twoPointsOrSmthn);
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        if (gamepad2.left_trigger > 0.5) {
            wally.setPosition(0.5);
            actual = onetwo;
            if (!index1){
                actual = 60;
            }
            intakeMotor.setPower(1);
            intakeMotor1.setPower(-1);
            if ((doos<5) && timer12.milliseconds()>200) {
                smart1.startRotate(actual);
                spoon +=1;
                //smart1 = new AxonRotator(hardwareMap,"spindexRoter", "slave", "smartTrack", reverse);
                //smart2.startRotate120(onetwo);
                index1 = true;
                timer12.reset();
            }
        } else if (gamepad2.right_trigger > 0.5) {
            Integralsum = 0;
            lasterror = 0;
            actual = onetwo;
            if (index1) {
                actual = 60;
            }
            TargetVelocity = getGoodVel(distance);
            if( TargetVelocity > 1800){
                TargetVelocity = 1400;
            }
            //TargetVelocity = 1400;
            shooterHood.setPosition(.35);
            double current = shoot2.getVelocity();
            shooterPower = PIDControl(TargetVelocity+100, current);
            shoot1.setPower(-shooterPower);
            shoot2.setPower(shooterPower);
            intakeMotor.setPower(1);
            intakeMotor1.setPower(-1);
            //artifactSpinner.setPower(-1);
                /*if (!pressShooter) {
                    pressShooter = true;
                    index1 = false;
                    timeBegan = System.currentTimeMillis();
                }*/
            //if (timeBegan + 500 < System.currentTimeMillis()) {

            if(shoot2.getVelocity()>(.92*TargetVelocity)){
                index1 = false;
                wally.setPosition(0);
                if (indexForSpindex == 0){
                    timer123.reset();
                    smart1.startRotate(actual);
                    spoon+=1;
                    //smart2.startRotate120(actual);
                    indexForSpindex += 1;
                }
                else if (indexForSpindex ==2 && timer123.seconds() > 0.35){
                    spoon+=1;
                    smart1.startRotate(actual);
                    //smart2.startRotate120(actual);
                    indexForSpindex = 0;
                }
                else if (indexForSpindex ==1 && timer123.seconds() > 0.2){
                    smart1.startRotate(actual);
                    spoon+=1;
                    //smart2.startRotate120(actual);
                    indexForSpindex += 1;
                }
                artifactPush.setPosition(kickUp);

                //gateShoot = false;
            }
            else {
                //wally.setPosition(.5);
                //artifactPush.setPosition(kickZero);
                //gateShoot = true;
            }
            //}
            //turretLeft.setPosition();
            //turretRight.setPosition();
        }
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
        else if (!gamepad2.y) {
            artifactPush.setPosition(kickZero);

            intakeMotor1.setPower(0);
            //artifactSpinner.setPower(0);
            shoot1.setPower(0);
            shoot2.setPower(0);
            pressShooter = false;
            artifactPush.setPosition(kickZero);
        }
        if (gamepad2.rightBumperWasPressed()) {
            if(scOOON()){
                sort = true;

            }
            else {
                scOOON();
                sort = false;
            }
        }
        /*if (gamepad2.aWasPressed()){
            if(numpurp == 2 && numgreen == 1){
                pattern = 1;
            }
        }
        if (gamepad2.bWasPressed()){
            if(numpurp == 2 && numgreen == 1) {
                pattern = 2;
            }
        }
        if (gamepad2.yWasPressed()){
            if(numpurp == 2 && numgreen == 1){
                pattern = 3;
            }
        }*/
        if (gamepad2.left_bumper) {
            //artifactSpinner.setPower(-1);
            intakeMotor.setPower(-1);
            intakeMotor1.setPower(1);
        }
        else {
            intakeMotor.setPower(0);
            intakeMotor.setPower(0);
        }
        if (gamepad2.dpadLeftWasPressed()) {
            smart1.startRotate(-actual);
            spoon+=1;
            //smart2.startRotate120(-actual);
        }
        if (gamepad2.dpadRightWasPressed()) {
            smart1.startRotate(actual);
            spoon+=1;
            //smart2.startRotate120(-actual);
        }
        if (gamepad2.dpadUpWasPressed()) {
            smart1.startRotate(-30);
            //smart2.startRotate120(-30);
        }
       /* if (gamepad2.yWasPressed()){
            spoon = 0;
            smart1 = new AxonRotator(hardwareMap,"spindexRoter", "slave", "smartTrack", reverse);
        }*/
        if (gamepad2.yWasPressed()){
            noLime = !noLime;
        }
        if (spoon >= 5){
            spoon =0;
            smart1 = new AxonRotator(hardwareMap,"spindexRoter", "slave", "smartTrack", reverse);
        }
       /*if (gamepad2.y){
            if (true){
                int tempPattern = actPattern;
                //pattern = 0;

                if (tempPattern == 1){
                    if (spindexColors[0]== purple && spindexColors[1] == purple && spindexColors[2] ==green){
                       if (state == 0 && acstate == 0){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                       else if (acstate == 1 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                       else  if (state == 0 && acstate == 2){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                        else if (acstate == 3 && timer1234.milliseconds()>500){
                           artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 4){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                    }
                    else if (spindexColors[0]==purple && spindexColors[1] == green && spindexColors[2] == purple){
                        if (state == 0 && acstate == 0){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                        else if (acstate == 1 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (acstate == 2 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                       else  if (state == 0 && acstate == 3){

                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                        else if (acstate == 4 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 5){
                            state = shootOneBall();
                            if (state == 1){
                                acstate = 6;
                            }
                            in=0;
                        }
                    }
                    else if (spindexColors[0]== green && spindexColors[1] == purple &&spindexColors[2] == purple){
                        if (acstate == 0 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 1){
                            state = shootOneBall();
                            if (state == 1){
                                acstate +=1;
                            }
                            in=0;
                        }else if (acstate == 2 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 3){
                            state = shootOneBall();
                            if (state == 1){
                                acstate +=1;
                            }
                            in=0;
                        }
                        else if (acstate == 4 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 5){
                            state = shootOneBall();
                            if (state == 1){
                                acstate = 6;
                            }
                            in=0;
                        }
                    }
                }

                if (tempPattern == 2){
                    if (spindexColors[0]==green && spindexColors[1] ==purple && spindexColors[2] == purple){
                        if (state == 0 && acstate == 0){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                        else if (acstate == 1 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else  if (state == 0 && acstate == 2){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                        else if (acstate == 3 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 4){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                    }
                    else if (spindexColors[0]==purple && spindexColors[1] == purple && spindexColors[2] == green){
                        if (acstate == 0 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        if (acstate == 1 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 2){
                            state = shootOneBall();
                            if (state == 1){
                                acstate +=1;
                            }
                            in=0;
                        }else if (acstate == 3 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 4){
                            state = shootOneBall();
                            if (state == 1){
                                acstate +=1;
                            }
                            in=0;
                        }
                        else if (acstate == 5 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 6){
                            state = shootOneBall();
                            if (state == 1){
                                acstate = 7;
                            }
                            in=0;
                        }
                    }
                    else if (spindexColors[0]==purple && spindexColors[1] == green && spindexColors[2] == purple){
                        if (acstate == 0 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 1){
                            state = shootOneBall();
                            if (state == 1){
                                acstate +=1;
                            }
                            in=0;
                        }else if (acstate == 2 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 3){
                            state = shootOneBall();
                            if (state == 1){
                                acstate +=1;
                            }
                            in=0;
                        }
                        else if (acstate == 4 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 5){
                            state = shootOneBall();
                            if (state == 1){
                                acstate = 6;
                            }
                            in=0;
                        }
                    }
                }

                if (tempPattern == 3){
                    if (spindexColors[0]==purple && spindexColors[1] == green && spindexColors[2] == purple){
                        if (state == 0 && acstate == 0){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                        else if (acstate == 1 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else  if (state == 0 && acstate == 2){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                        else if (acstate == 3 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 4){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                    }
                    else if (spindexColors[0]==purple && spindexColors[1] == purple && spindexColors[2] == green){
                        if (state == 0 && acstate == 0){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                        else if (acstate == 1 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (acstate == 2 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else  if (state == 0 && acstate == 3){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                        else if (acstate == 4 && timer1234.milliseconds()>500){

                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (state == 0 && acstate == 5){
                            state = shootOneBall();
                            if (state == 1){
                                acstate = 6;
                            }
                            in=0;
                        }
                    }
                    else if (spindexColors[0]==green && spindexColors[1] == purple && spindexColors[2] == purple){
                        if (acstate == 0 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else  if (state == 0 && acstate == 1){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                        else if (acstate == 2 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                       else  if (acstate == 3 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else  if (state == 0 && acstate == 4){
                            state = shootOneBall();
                            if (state == 1){
                                acstate+=1;
                            }
                            in=0;
                        }
                        else if (acstate == 5 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else if (acstate == 6 && timer1234.milliseconds()>500){
                            artifactPush.setPosition(kickZero);
                            rotateSpindex();
                            timer1234.reset();
                            acstate+=1;
                            state = 0;
                        }
                        else  if (state == 0 && acstate == 7){
                            state = shootOneBall();
                            if (state == 1){
                                acstate = 8;
                            }
                            in=0;
                        }
                    }
                }
            }
        }
       else{
           state = 0;
           acstate = 0;
       }*/


        telemetry.addData("see tag? ", limelight3A.getLatestResult());
        //telemetry.addData("hood position", shooterHood.getPosition());
        // telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity2", shoot2.getVelocity());
        telemetry.addData("velocity1", shoot1.getVelocity());
        //telemetry.addData("power", shooterPower);
        telemetry.addData("automatedDrive", automatedDrive);
        telemetry.addData("distance", distance);
        // telemetry.addData("left pos", turretLeft.getPosition());
        //telemetry.addData("right pos", turretRight.getPosition());
        //telemetry.addData("spindexerPos", smart1.printAbsoluteAngle());
        //telemetry.addData("voltage", smart1.getVol());
        telemetry.addData("servTargPos", smart1.getTargetPosition());
        telemetry.addData("servCurrrPos", smart1.getCurrentPosition());
        telemetry.addData("TargetVel", TargetVelocity);
        /*telemetry.addData("indexforspindex", indexForSpindex);
        telemetry.addData("time123", timer123.seconds());
        telemetry.addData("index1", index1);*/
        telemetry.addData("distance (cm)", dis.getDistance(DistanceUnit.CM));
       /* telemetry.addData("pattern", actPattern);
        telemetry.addData("turret", turretLeft.getPosition());
        telemetry.addData("in", in);*/

        //telemetry.addData("fuckDelta", smart1.fuckDelta());
        telemetry.update();
    }
}