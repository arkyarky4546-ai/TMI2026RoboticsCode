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
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.AxonRotator;

import com.qualcomm.hardware.limelightvision.LLResult;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

@Configurable
@TeleOp
public class ImprovedTeleOp6221Red extends OpMode {
    private Follower follower;
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
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime timer12 = new ElapsedTime();
    ElapsedTime timer123 = new ElapsedTime();
    double lasterror = 0;
    double limelightPause;
    int index = 0;
    int sixty = 60;
    int Nsixty = -60;
    int onetwo = 120;
    int Nonetwo = -115;
    int actual = -115;
    Servo wally;
    float time = System.currentTimeMillis();
    double lastTimeFrame = 0;
    double currTimeFrame = 0;
    int indexForSpindex = 0;
    boolean gateShoot = false;
    double turTurn = .4;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(6);
        startingPose = new Pose(8, -96, Math.toRadians(0));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");
        intakeGate = hardwareMap.get(Servo.class, "intakeGate");

        //artifact storage
        // artifactSpinner = hardwareMap.get(CRServo.class, "spindexRoter");
        artifactPush = hardwareMap.get(Servo.class, "push");

        //shooter
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turretRight.setPosition(turTurn);
        turretLeft.setPosition(turTurn);
        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        wally = hardwareMap.get(Servo.class, "wally");
        // smart = hardwareMap.get(CRServo.class, "spindexRoter");
        smart1 = new AxonRotator(hardwareMap,"spindexRoter","slave", "smartTrack", reverse);
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterHood = hardwareMap.get(Servo.class, "shooterHood");
        shooterHood.setPosition(0.62);
        aiming = true;
        artifactPush.setPosition(kickZero);
        shooterPower = 1;
        //smart1.startRotate120(right);
        timer12.reset();
        timer123.reset();
        timer1.reset();
        indexForSpindex = 0;
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(8, -96))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();
    }
    public Pose altitudeExist(Pose currentPose){
        /*double x = Math.abs(currentPose.getY());
        double y = currentPose.getX();
        double verti = Math.abs(x-currentPose.getX());
        double hori = Math.abs(y-Math.abs(currentPose.getY()));
        double hypo = Math.sqrt(verti*verti + hori*hori);
        double sinTheta = verti/hypo;
        double cosTheta = hori/hypo;
        //double altitude = sinTheta* verti;
        double vertiAlti = cosTheta * verti;
        double newVerti = sinTheta * vertiAlti;
        double newHori = cosTheta * vertiAlti;
        return new Pose((currentPose.getX()+newVerti), -(Math.abs(currentPose.getX()+newHori)), Math.toRadians(-47));*/
        double value = Math.abs(Math.abs(currentPose.getX()) + Math.abs(currentPose.getY()))/2;
        //telemetry.update();
        return new Pose(value, -value, Math.toRadians(-47));
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

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();

        limelight3A.start();
    }

    @Override
    public void loop() {
        smart1.update(telemetry);
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
        if(result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
            for(LLResultTypes.FiducialResult tag: results){
                if(tag.getFiducialId() == 24){
                    telemetry.addData("here", tag.getFiducialId());
                    double tx = tag.getTargetXDegrees();
                    distance = result.getTa();
                    /*double turnCmd = TURN_Constant;
                    if (tx<0){
                        turnCmd = -TURN_Constant;
                    }
                    double turn =turretLeft.getPosition() + turnCmd;
                    if(turn<0.005){
                        turn=0.994;
                    }
                    if(turn>0.995){
                        turn=0.006;
                    }

                    if (tx > 10 || tx < -10) {
                        turretLeft.setPosition(turn);
                        turretRight.setPosition(turn);
                    }*/
                }
            }
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
            telemetry.addData("x: ", follower.getPose().getX());
            telemetry.addData("y: ", follower.getPose().getY());
            telemetry.addData("going to x: ", targetPose.getX());
            telemetry.addData("going to y: ", targetPose.getY());
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
            actual = Nonetwo;
            if (!index1){
                actual = -60;
            }
            intakeMotor.setPower(1);
            intakeMotor1.setPower(-1);
            if (timer12.milliseconds()>100) {
                time = System.currentTimeMillis();
                smart1.startRotate(actual);
                index1 = true;
                timer12.reset();
            }
        } else if (gamepad2.right_trigger > 0.5) {
            Integralsum = 0;
            lasterror = 0;
            actual = Nonetwo;
            if (index1) {
                actual = -60;
            }
            TargetVelocity = getGoodVel(distance);
            //TargetVelocity = 1400;
            shooterHood.setPosition(.4);
            double current = shoot2.getVelocity();
            shooterPower = PIDControl(TargetVelocity, current);
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
            //if (timeBegan + 300 < System.currentTimeMillis()) {

            if(shoot2.getVelocity()>(.92*TargetVelocity)){
                index1 = false;
                wally.setPosition(0);
                if (indexForSpindex == 0){
                    timer123.reset();
                    smart1.startRotate(actual);
                    indexForSpindex += 1;
                }
                else if (indexForSpindex ==2 && timer123.seconds() > 0.35){
                    smart1.startRotate(actual);
                    indexForSpindex = 0;
                }
                else if (indexForSpindex ==1 && timer123.seconds() > 0.2){
                    smart1.startRotate(actual);
                    indexForSpindex += 1;
                }
                artifactPush.setPosition(kickUp);

                //gateShoot = false;
            }
            else {
                wally.setPosition(.3);
                //artifactPush.setPosition(kickZero);
                //gateShoot = true;
            }
            //}
            //turretLeft.setPosition();
            //turretRight.setPosition();
        }
        else {
            artifactPush.setPosition(kickZero);
            intakeMotor.setPower(0);
            intakeMotor1.setPower(0);
            //artifactSpinner.setPower(0);
            shoot1.setPower(0);
            shoot2.setPower(0);
            pressShooter = false;
            artifactPush.setPosition(kickZero);
        }
        if (gamepad2.right_bumper) {
            //artifactSpinner.setPower(1);
        }
        if (gamepad2.left_bumper) {
            //artifactSpinner.setPower(-1);
            intakeMotor.setPower(-1);
            intakeMotor1.setPower(1);
        }
        if (gamepad2.dpadLeftWasPressed()) {
            smart1.startRotate(actual);
        }
        if (gamepad2.dpadRightWasPressed()) {
            smart1.startRotate(actual);
        }


        // telemetry.addData("see tag? ", limelight3A.getLatestResult());
        telemetry.addData("hood position", shooterHood.getPosition());
        // telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", shoot2.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);
        // telemetry.addData("left pos", turretLeft.getPosition());
        //telemetry.addData("right pos", turretRight.getPosition());
        //telemetry.addData("spindexerPos", smart1.printAbsoluteAngle());
        //telemetry.addData("voltage", smart1.getVoltage());
        telemetry.addData("servTargPos", smart1.getTargetPosition());
        telemetry.addData("servCurrrPos", smart1.getCurrentPosition());
        telemetry.addData("TargetVel", TargetVelocity);
        telemetry.addData("indexforspindex", indexForSpindex);
        telemetry.addData("time123", timer123.seconds());
        telemetry.addData("index1", index1);
        //telemetry.addData("fuckDelta", smart1.fuckDelta());
        telemetry.update();
    }
}