/*package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutoAimTurret;
import org.firstinspires.ftc.teamcode.AutoTurret;
import org.firstinspires.ftc.teamcode.LimeLight;
import org.firstinspires.ftc.teamcode.ShooterConstants;
import org.firstinspires.ftc.teamcode.intakeShoot;
import org.firstinspires.ftc.teamcode.shooterThread;

@Configurable
@TeleOp
public class blueTeleOp extends OpMode {
    // newly created classes
    Drivetrain drivetrain; //all driving functionality and pedro pathing

    /* switched back to autoAimTurret for the scrimmage)
    //AutoTurret turret; //autoaiming and manual control
    AutoAimTurret turret;


    intakeShoot shooterAndIntake;
    //ShooterAndIntake shooterAndIntake; //everything else really - - there wasn't a good way to split them up bc all the parts are the same

    Limelight3A limelight3A; //for pattern recognition
    boolean patternDetected = false;
    boolean shootFirst = true;
    boolean aim = true;
    LimeLight Lime;
    int[] pattern = {1,2,2};
    int[] ppg = {2,2,1};
    int[] pgp = {2,1,2};
    int[] gpp = {1,2,2};

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setModeBlue();
        Lime = new LimeLight(hardwareMap);

        //switched back
        // turret = new AutoTurret(hardwareMap, "turretLeft", "turretRight");
        turret = new AutoAimTurret(hardwareMap, "turretLeft", "turretRight");
        turret.setModeBlue();

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(6);
        limelight3A.start();
        shooterAndIntake = new intakeShoot(hardwareMap,"intake", "intake1",
                "shoot1", "shoot2",
                "spindexRoter", "slave",
                "wally", "color1", "color2", "shooterHood", drivetrain.getFollower());
        //shooterAndIntake = new ShooterAndIntake(hardwareMap);
        shooterAndIntake.setModeBlue();

    }

    @Override
    public void loop() {
        //drivetrain calls - all controlled by gamepad1
        // (a = reset pedro pose, x = altitudes exist, b = abort pedro path)
        drivetrain.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.xWasPressed(), gamepad1.bWasPressed(), gamepad1.yWasPressed(), gamepad1.dpadRightWasPressed(), gamepad1.dpadLeftWasPressed());
        if(gamepad1.a){
            drivetrain.resetCurrentPose();
        }
        if(gamepad1.b){
            drivetrain.resetCurrentPoseGoal();
        }
        if(gamepad1.dpadLeftWasPressed()) { //shootFar
            shooterAndIntake.setShootFar();
            aim = false;
            turret.setManualPosition(0.38); //red = 0.81 and everything else should be the same
        }

        //turret calls - manual is controlled by gamepad2 on the dpad
        // (down = autoaiming on/off, up = set center pos, left/right = manual turning

        //switched back
        //turret.updateAuto(drivetrain.getFollower(), telemetry, shooterAndIntake.turretAngle(), aim);
        if(aim){
            turret.update(drivetrain.getFollower(), telemetry);
        }

        if(gamepad1.dpadDownWasPressed()){
            aim = !aim;
            if(Lime.getPatternFromLimelight() == 0){
                pattern = gpp;

            }
            else if(Lime.getPatternFromLimelight() == 1){
                pattern = pgp;

            }
            else if(Lime.getPatternFromLimelight() == 2){
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

        //shooter and intake calls - controlled by gamepad2
        //(left trigger = intake, left bumper = out intake, right trigger = shoot)
        boolean leftTrigger = false, rightTrigger = false;
        if(gamepad2.left_trigger > 0.75){
            leftTrigger = true;
        }
        if(gamepad2.right_trigger > 0.75){
            if(shootFirst){
                //shooterAndIntake.resetPID();
            }
            rightTrigger = true;
            shootFirst = false;
            drivetrain.setHoldMode(true);
        }

        else{
            shootFirst = true;
            drivetrain.setHoldMode(false);
        }

        telemetry.addData("rightTrigger", rightTrigger);
        telemetry.addData("shootfirst", shootFirst);
        telemetry.addData("target velocity", shooterAndIntake.getVelocity());
        //telemetry.addData("shooter1 velocity", shooterAndIntake.shoot1.getVelocity());
        //telemetry.addData("shooter2 velocity", shooterAndIntake.shoot2.getVelocity());

        //telemetry.addData("recoil", shooterAndIntake.recoil);
        //telemetry.addData("hood", shooterAndIntake.shooterHood.getPosition());
        telemetry.addData("distance", drivetrain.getDistanceFromGoal());
        telemetry.addData("turret angle", shooterAndIntake.turretAngle());
        //telemetry.addData("turret value", turret.getTargetPos());
        // telemetry.update();
        // shooterAndIntake.update(leftTrigger ,(rightTrigger || gamepad2.right_bumper), gamepad2.left_bumper, gamepad2.dpadUpWasPressed(),telemetry, gamepad2.right_bumper, gamepad2.xWasPressed(), pattern, drivetrain.getFollower());
        shooterAndIntake.update(leftTrigger, gamepad2.left_bumper, (rightTrigger || gamepad2.right_bumper), drivetrain.getFollower());
        //public void update(boolean intakeActive, boolean intakeOut, boolean shootActive, Follower follower) {

        //shooterAndIntake.update(drivetrain.getDistanceFromGoal(), leftTrigger, rightTrigger, gamepad2.left_bumper, gamepad2.right_bumper, gamepad2.x, gamepad2.b, gamepad2.y, gamepad1.dpadRightWasPressed(), gamepad1.dpadUpWasPressed(), telemetry);

    }
    @Override
    public void stop(){
        shooterAndIntake.stopT();
    }
}*/
