package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
//@TeleOp
public class autoaimingTest extends OpMode {
    ElapsedTime timer =new ElapsedTime();
    Limelight3A limelight3A;
    Drivetrain drivetrain;
    Servo turretRight;
    Servo turretLeft;
    double TURN_Constant = 0.1;
    boolean goingLeft;
    boolean goingRight;
    double limelightPause = 0;
    int index = 0;
    boolean toFarLeft = false;
    boolean toFarRight = false;
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(6);
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        timer.reset();
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            turretLeft.setPosition(0);
            turretRight.setPosition(0);
        }
        if (index == 0) {
            limelightPause = System.currentTimeMillis();
            index = 67; // I am an adult
        }
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;
        drivetrain.arcadeDrive(-drive, strafe, twist);
        if(twist < -0.1){
            goingLeft = true;
            goingRight = false;
        }
        else if(twist > 0.1){
            goingLeft = false;
            goingRight = true;
        }
        double goingToBeSetPos = -67;
        LLResult result = limelight3A.getLatestResult();
        if(result != null && result.isValid()){
            if(timer.milliseconds()>1000) {
                double tx = result.getTx();
                double turnCmd = TURN_Constant;
                if (tx < 0) {
                    turnCmd = -TURN_Constant;
                }
                double turn = turretLeft.getPosition() + turnCmd;
                if (turn < 0.005) {
                    toFarRight = true;
                    //turn = 0.994;
                }
                if (turn > 0.995) {
                    toFarLeft = true;
                    //turn = 0.006;
                }

                if(toFarRight){ //going to 0.995
                    if(turretLeft.getPosition() > 0.7){
                        turn = 0.995;
                        toFarRight = false;
                    }
                    else{
                        turn = turretLeft.getPosition() + 0.2;
                    }
                }
                if(toFarLeft){
                    if(turretLeft.getPosition() < 0.2){
                        turn = 0.005;
                        toFarLeft = false;
                    }
                    else{
                        turn = turretLeft.getPosition() - 0.2;
                    }
                }

                if (tx > 10 || tx < -10) {
                    turretLeft.setPosition(turn);
                    turretRight.setPosition(turn);
                }
                telemetry.addData("tx", result.getTx());
                telemetry.addData("turn ", turn);
                timer.reset();
            }
        }
            else{ //cant see it
            if(timer.milliseconds()>1000) {
                if (goingLeft) { //spinning left - setting more right
                    goingToBeSetPos = turretLeft.getPosition() - (1 * TURN_Constant);
                } else if (goingRight) {
                    goingToBeSetPos = turretLeft.getPosition() + (1 * TURN_Constant);
                } else {
                    goingToBeSetPos = turretLeft.getPosition() - (1 * TURN_Constant);
                }
                if (goingToBeSetPos < 0.005) {
                    toFarRight = true;
                    //goingToBeSetPos = 0.995;
                } else if (goingToBeSetPos > 0.995) {
                    toFarLeft = true;
                    //goingToBeSetPos = 0.005;
                }
                if (toFarRight) { //going to 0.995
                    if (turretLeft.getPosition() > 0.9) {
                        goingToBeSetPos = 0.995;
                        toFarRight = false;
                    } else {
                        goingToBeSetPos = turretLeft.getPosition() + 0.05;
                    }
                }
                if (toFarLeft) {
                    if (turretLeft.getPosition() < 0.1) {
                        goingToBeSetPos = 0.005;
                        toFarLeft = false;
                    } else {
                        goingToBeSetPos = turretLeft.getPosition() - 0.05;
                    }
                }
                timer.reset();
            }
                turretLeft.setPosition(goingToBeSetPos);
                turretRight.setPosition(goingToBeSetPos);
                telemetry.addData("goingToBeSetPos", goingToBeSetPos);
            }
        telemetry.addData("isValid ", result.isValid());
        if(result.isValid()){
            telemetry.addData("tag?, Yes",0);
        }
        else{
            telemetry.addData("tag?, nope", 0);
        }
        telemetry.addData("left ", goingLeft);
        telemetry.addData("right ", goingRight);
        telemetry.addData("tofarleft ", toFarLeft);
        telemetry.addData("tofarright ", toFarRight);
        telemetry.addData("timer", timer.milliseconds());
        telemetry.update();
    }
}
