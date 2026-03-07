/*package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutoAimTurret;
import org.firstinspires.ftc.teamcode.LimeLight;
import org.firstinspires.ftc.teamcode.intakeShoot;

@Configurable
@TeleOp
public class blueTeleopBeast extends OpMode {


    public static double Kp = 0.0121;
    public static double Ki = 0.00014;
    public static double Kd = 0.0000;
    public static double Kf = 0.0000;
    public static double TargetVelocity = 1400;

    public static double hoodPos = 0.4;
    public static double turretPos = 0.8;
    public static double recoil = 0.03;
    public static double wallPos = 0.0;


    Drivetrain drivetrain; //all driving functionality and pedro pathing
    AutoAimTurret turret;
    intakeShoot shooterAndIntake;
    Limelight3A limelight3A; //for pattern recognition
    LimeLight Lime;

    boolean shootFirst = true;
    boolean aim = true;
    int[] pattern = {1,2,2};
    int[] ppg = {2,2,1};
    int[] pgp = {2,1,2};
    int[] gpp = {1,2,2};

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setModeBlue();
        Lime = new LimeLight(hardwareMap);

        turret = new AutoAimTurret(hardwareMap, "turretLeft", "turretRight");
        turret.setModeBlue();

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(6);
        limelight3A.start();

        shooterAndIntake = new intakeShoot(hardwareMap,"intake", "intake1",
                "shoot1", "shoot2",
                "spindexRoter", "slave",
                "wally", "color1", "color2", "shooterHood", drivetrain.getFollower());
        shooterAndIntake.setModeBlue();

        shooterAndIntake.wallPos(0.2);
    }

    @Override
    public void loop() {
        drivetrain.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.xWasPressed(), gamepad1.bWasPressed(), gamepad1.yWasPressed(), gamepad1.dpadRightWasPressed(), gamepad1.dpadLeftWasPressed());
        if(gamepad1.a){
            drivetrain.resetCurrentPose();
        }
        if(gamepad1.b){
            drivetrain.resetCurrentPoseGoal();
        }
        if(gamepad1.dpadLeftWasPressed()) {
            shooterAndIntake.setShootFar();
            aim = false;
            turret.setManualPosition(0.38);
        }


        if(aim){
            turret.update(drivetrain.getFollower(), telemetry);
        }

        if(gamepad1.dpadDownWasPressed()){
            aim = !aim;
            if(Lime.getPatternFromLimelight() == 0){
                pattern = gpp;
            } else if(Lime.getPatternFromLimelight() == 1){
                pattern = pgp;
            } else if(Lime.getPatternFromLimelight() == 2){
                pattern = ppg;
            }
        }
        else if(!aim && gamepad1.dpad_left){
            turret.manualLeft();
        }
        else if(!aim && gamepad1.dpad_right){
            turret.manualRight();
        }
        else if(gamepad1.dpadUpWasPressed()){
            turret.setCenter();
            aim = !aim;
        }


        boolean leftTrigger = false, rightTrigger = false;
        if(gamepad1.left_trigger > 0.75){
            leftTrigger = true;
        }

        if(gamepad1.right_trigger > 0.75){
            rightTrigger = true;
            shootFirst = false;
            drivetrain.setHoldMode(true);

            shooterAndIntake.shootsetVelocity(TargetVelocity);
        } else {
            shootFirst = true;
            drivetrain.setHoldMode(false);
            shooterAndIntake.wallPos(0.1);
        }

        shooterAndIntake.hoodPos(hoodPos);

        shooterAndIntake.update(leftTrigger, gamepad1.left_bumper, rightTrigger, gamepad1.right_bumper, drivetrain.getFollower(), telemetry);

        telemetry.addData("RightTrigger (Shooting)", rightTrigger);
        telemetry.addData("Target Velocity", TargetVelocity);
        telemetry.addData("Current Velocity", shooterAndIntake.getVelocity());
        telemetry.addData("Distance", drivetrain.getDistanceFromGoal());
        telemetry.addData("Turret Angle", shooterAndIntake.turretAngle());
    }

    @Override
    public void stop(){
        shooterAndIntake.stopT();
    }
}*/