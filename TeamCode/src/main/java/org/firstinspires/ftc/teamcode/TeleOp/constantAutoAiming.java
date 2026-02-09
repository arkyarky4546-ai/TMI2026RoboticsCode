package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Configurable
//@TeleOp
public class constantAutoAiming extends OpMode {
    Limelight3A limelight3A;
    Drivetrain drivetrain;
    Servo turretRight;
    Servo turretLeft;
    double TURN_Constant = 0.005;
    boolean goingLeft;
    boolean goingRight;
    double limelightPause = 0;
    int index = 0;
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
    }

    @Override
    public void loop() {
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
        if (limelightPause + 120 < System.currentTimeMillis()){
            limelightPause = System.currentTimeMillis();
        if(result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
            for(LLResultTypes.FiducialResult tag: results){
                if(tag.getFiducialId() == 24){
                    telemetry.addData("here", tag.getFiducialId());
                    double tx = tag.getTargetXDegrees();
                    double turnCmd = TURN_Constant;
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
                    }
                }
            }
        }
        else{ //cant see it
            double currentPos = turretLeft.getPosition();
            goingToBeSetPos = turretLeft.getPosition();
            if(goingLeft){ //spinning left - setting more right
                goingToBeSetPos = turretLeft.getPosition() - (2 * TURN_Constant);
            }
            else if(goingRight){
                goingToBeSetPos = turretLeft.getPosition() + (2 * TURN_Constant);
            }
            else{
                goingToBeSetPos = turretLeft.getPosition() - (2 * TURN_Constant);
            }
            if(goingToBeSetPos < 0.005){
                goingToBeSetPos = 0.995;
            }
            else if(goingToBeSetPos > 0.995){
                goingToBeSetPos = 0.005;
            }
            turretLeft.setPosition(goingToBeSetPos);
            turretRight.setPosition(goingToBeSetPos);

        }}
        telemetry.addData("isValid ", result.isValid());
        if(result.isValid()){
            telemetry.addData("tag?", result.getFiducialResults());
        }
        else{
            telemetry.addData("tag?, nope", 0);
        }
        telemetry.addData("left ", goingLeft);
        telemetry.addData("right ", goingRight);
        telemetry.addData("goingToBeSetPos", goingToBeSetPos);
        telemetry.update();
    }

    public void start(){
        limelight3A.start();
    }
}
