package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Import your new move-shoot classes
import org.firstinspires.ftc.teamcode.AutoTurret;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.ShooterConstants;
import org.firstinspires.ftc.teamcode.shooterThread;
import org.firstinspires.ftc.teamcode.LimeLight;
import org.firstinspires.ftc.teamcode.intakeShoot;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp
public class blueTeleopFlanker extends OpMode {

    public static double Kp = 0.0121;
    public static double Ki = 0.00014;
    public static double Kd = 0.0000;
    public static double Kf = 0.0000;

    public static double turretPos = 0.8;
    public static double recoil = 0.03;
    public static double wallPos = 0.0;


    // --- ROBOT SUBSYSTEMS ---
    Drivetrain drivetrain;
    AutoTurret turret;
    intakeShoot shooterAndIntake;
    Limelight3A limelight3A;
    LimeLight Lime;


    Shooter shooterCalculator;
    //shooterThread shootThread;
    private double greenPos = 0.0;

    boolean shootFirst = true;
    int index = 1;
    boolean aim = true;
    int[] pattern = {1,2,2};
    int[] ppg = {2,2,1};
    int[] pgp = {2,1,2};
    int[] gpp = {1,2,2};
    int[][] entire = {ppg, pgp, gpp};
    private boolean reset = true;
    private int patternIndex = 0;
    private boolean jorkIt = false;
    private boolean sort = false;
    private boolean sorting = false;
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime spin = new ElapsedTime();
    private boolean patternFound = false;
    private Servo turretRight;
    boolean slow = false;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setModeBlue();
        Lime = new LimeLight(hardwareMap);
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        // Initialize the new AutoTurret
        turret = new AutoTurret(hardwareMap, "turretLeft", "turretRight");
        turret.setModeBlue();

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(6);
        limelight3A.start();

        shooterAndIntake = new intakeShoot(hardwareMap,"intake", "intake1",
                "shoot1", "shoot2",
                "spindexRoter", "slave",
                "wally", "color1", "color2", "shooterHood", drivetrain.getFollower());
        shooterAndIntake.setModeBlue();

        shooterCalculator = new Shooter();
        double initialHeading = drivetrain.getFollower().getPose().getHeading();
        //shootThread = new shooterThread(shooterCalculator, drivetrain.getFollower(), ShooterConstants.GOAL_POSE_BLUE, initialHeading);
        //shootThread.start();

        //shooterAndIntake.wallPos(0.2);

    }

    @Override
    public void loop() {
        // --- DRIVETRAIN ---
        drivetrain.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.xWasPressed(), gamepad2.bWasPressed(), gamepad1.yWasPressed(), gamepad2.dpadRightWasPressed(), gamepad2.dpadLeftWasPressed(), gamepad1.dpad_left, slow);

        if(gamepad1.a){
            drivetrain.resetCurrentPose();
        }
        if(gamepad1.b){
            drivetrain.resetCurrentPoseGoal();
        }
        if(gamepad2.yWasPressed()){
            sort = true;
            greenPos = shooterAndIntake.getGreen();
        }
        if(gamepad2.a){
            sort = true;
            sorting = true;
            reset = false;
            if(spin.milliseconds() < 500) {
                shooterAndIntake.slow(index);
            }
            else{
                spin.reset();
                if(index > 5){
                    index = 0;
                }
                else{
                    index +=1;
                }
            }
        }
        if(gamepad2.xWasPressed()){
            //sort  = false;
            reset = false;
            if(sorting){
                sorting = false;
                shootTimer.reset();
            }
            shooterAndIntake.colorSort(greenPos, pattern);


        }
       /* if (!sorting && shootTimer.milliseconds() > 1000 && shootTimer.milliseconds() < 1050){
            shooterAndIntake.colorSort(greenPos, pattern);
        }*/

        double currentHeading = drivetrain.getFollower().getPose().getHeading();
        //  shootThread.update(drivetrain.getFollower(), ShooterConstants.GOAL_POSE_BLUE, currentHeading);

       /* double dynamicTurretAngle = shootThread.getTurretPos();
        double dynamicHoodPos = shootThread.getHoodPos();
        double dynamicFlywheelSpeed = shootThread.getSpeed();*/

        // Pass the dynamically calculated angle into the turret
        if(aim){
            turret.updateAuto(drivetrain.getFollower(), telemetry, shooterAndIntake.turretAngle(), aim);
        }

        if(gamepad2.dpadDownWasPressed()){
            aim = !aim;
        }
        if(!patternFound){
            if(Lime.getPatternFromLimelight() == 0){
                pattern = gpp;
                patternFound = true;
            } else if(Lime.getPatternFromLimelight() == 1){
                pattern = pgp;
                patternFound = true;
            } else if(Lime.getPatternFromLimelight() == 2){
                pattern = ppg;
                patternFound = true;
            }
        }
        else if(gamepad1.dpadRightWasPressed()){
            slow = !slow;
        }

        boolean leftTrigger = false;
        boolean rightTrigger = false;

        boolean gateLeftBumper = gamepad1.left_bumper;
        if(gamepad2.leftBumperWasPressed()){
            if(patternIndex >2){
                patternIndex  = 0;
            }
            else{
                patternIndex += 1;
            }
            pattern = entire[patternIndex];
        }

        boolean rightBumper = gamepad2.right_bumper;


        //Check for Shooting (Highest Priority)
        if(gamepad1.right_trigger > 0.75){
            reset = true;
            rightTrigger = true;
            shootFirst = false;
            sort = false;

            // Feed the dynamically calculated velocity instead of the static TargetVelocity
            //shooterAndIntake.shootsetVelocity(dynamicFlywheelSpeed);

            // Override and disable intake while shooting

        } else {
            // Not Shooting (Allow Intake and Default States)
            shootFirst = true;
            //shooterAndIntake.wallPos(.2127);
            if(reset) {
                //shooterAndIntake.setPos(0, 0);
            }
            // Only allow intake if the right trigger is NOT pressed
            if(gamepad1.left_trigger > 0.75){
                reset = true;
                leftTrigger = true;
            }
        }

        //  shooterAndIntake.hoodPos(dynamicHoodPos);

        shooterAndIntake.update(leftTrigger, false, gateLeftBumper, rightTrigger, rightBumper, drivetrain.getFollower(), telemetry, sort, gamepad2.dpad_up, false);


        telemetry.addData("RightTrigger (Shooting)", rightTrigger);
        telemetry.addData("sort", sort);
        telemetry.addData("turret angle", turretRight.getPosition());
        //telemetry.addData("ShootFirst", shootFirst)
        //    telemetry.addData("Target Velocity (Dynamic)", dynamicFlywheelSpeed);
        telemetry.addData("Current Velocity", shooterAndIntake.getVelocity());
        //  telemetry.addData("Dynamic Hood Pos", dynamicHoodPos);
        telemetry.addData("Distance", drivetrain.getDistanceFromGoal());
        //telemetry.addData("Calculated Turret Angle", dynamicTurretAngle);
        telemetry.addData("Debug Mode", rightBumper);
        // telemetry.addData("BL Vertex Has Ball", shooterAndIntake.getBLBallState());
        //  telemetry.addData("ST Vertex Has Ball", shooterAndIntake.getSTBallState());
        telemetry.addData("Pattern Found: ", patternFound);
        if(pattern[0] == ppg[0] && pattern[1] == ppg[1] && pattern[2] == ppg[2]){
            telemetry.addData("Pattern: ppg", 0);
        }
        if(pattern[0] == pgp[0] && pattern[1] == pgp[1] && pattern[2] == pgp[2]){
            telemetry.addData("Pattern: pgp", 0);
        }
        if(pattern[0] == gpp[0] && pattern[1] == gpp[1] && pattern[2] == gpp[2]) {
            telemetry.addData("Pattern: gpp", 0);
        }
        telemetry.addData("greenPos: ", greenPos);
    }

    @Override
    public void stop(){
        // Crucial: Kill the background thread to prevent memory leaks or crashes after stopping the OpMode
        //shootThread.stopThread();
        shooterAndIntake.stopT();

    }
}