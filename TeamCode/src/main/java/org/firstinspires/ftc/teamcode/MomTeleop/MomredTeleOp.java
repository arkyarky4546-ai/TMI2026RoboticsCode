package org.firstinspires.ftc.teamcode.MomTeleop;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@Configurable
// @TeleOp
public class MomredTeleOp extends OpMode {
    /* newly created classes */
    Drivetrain drivetrain; //all driving functionality and pedro pathing
    Turret turret; //autoaiming and manual control
    //ShooterandIntake shooterAndIntake; //everything else really - - there wasn't a good way to split them up bc all the parts are the same

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setModeRed();

        turret = new Turret(hardwareMap, "turretLeft", "turretRight");
        turret.setModeRed();

        //shooterAndIntake = new ShooterandIntake(hardwareMap);

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
        boolean leftTrigger = false, rightTrigger = false;
        if(gamepad2.left_trigger > 0.75){
            leftTrigger = true;
        }
        if(gamepad2.right_trigger > 0.75){
            rightTrigger = true;
        }
        telemetry.addData("rightTrigger", rightTrigger);
        //shooterAndIntake.update(drivetrain.getDistanceFromGoal(), leftTrigger, rightTrigger, gamepad2.left_bumper, gamepad2.right_bumper, gamepad2.x, gamepad2.b, telemetry);

    }
}
