package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.revisedTeleOp.Turret;
import org.firstinspires.ftc.teamcode.revisedTeleOp.Drivetrain;

import java.util.List;

@Configurable
@TeleOp
public class redTeleOp extends OpMode {
    /* newly created classes */
    Drivetrain drivetrain; //all driving functionality and pedro pathing
    Turret turret; //autoaiming and manual control
    ShooterAndIntake shooterAndIntake; //everything else really - - there wasn't a good way to split them up bc all the parts are the same

    Limelight3A limelight3A; //for pattern recognition
    boolean patternDetected = false;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setModeRed();

        turret = new Turret(hardwareMap, "turretLeft", "turretRight");
        turret.setModeRed();

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(6);
        limelight3A.start();

        shooterAndIntake = new ShooterAndIntake(hardwareMap);

    }

    @Override
    public void loop() {
        //drivetrain calls - all controlled by gamepad1
        // (a = reset pedro pose, x = altitudes exist, b = abort pedro path)
        drivetrain.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.xWasPressed(), gamepad1.bWasPressed());
        if(gamepad1.a){
            drivetrain.resetCurrentPose();
        }

        //turret calls - manual is controlled by gamepad2 on the dpad
        // (down = autoaiming on/off, up = set center pos, left/right = manual turning)
        turret.update(drivetrain.getFollower(), telemetry);
        if(gamepad2.dpadDownWasPressed()){
            turret.switchMode();
        }
        if(gamepad2.dpad_up){
            turret.setTurretCenter();
        }
        else if(!turret.isAutoAiming() && gamepad2.dpad_left){
            turret.manualLeft();
        }
        else if(!turret.isAutoAiming() && gamepad2.dpad_right){
            turret.manualRight();
        }

        //shooter and intake calls - controlled by gamepad2
        //(left trigger = intake, left bumper = out intake, right trigger = shoot)
        shooterAndIntake.update(drivetrain.getDistanceFromGoal(), (gamepad2.left_trigger > 0.25), (gamepad2.right_trigger > 0.25), gamepad2.left_bumper, gamepad2.right_bumper);

        //limelight pattern detection
        if(!patternDetected) {
            LLResult result = limelight3A.getLatestResult();
            if (result != null && result.isValid()){
                List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
                for (LLResultTypes.FiducialResult tag : results) {
                    if (tag.getFiducialId() == 23) {
                        shooterAndIntake.setPattern(1);
                        patternDetected = true;
                    } else if (tag.getFiducialId() == 22) {
                        shooterAndIntake.setPattern(2);
                        patternDetected = true;
                    } else if (tag.getFiducialId() == 21) {
                        shooterAndIntake.setPattern(3);
                        patternDetected = true;
                    }
                }
            }
        }
    }
}
