package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class shootingReg extends OpMode {
    Drivetrain drivetrain;

    DcMotorEx shoot1;
    DcMotorEx shoot2;

    private Follower follower;

    private final Pose startPose = new Pose(128,-26, Math.toRadians(47));

    double distance;
    double shootingVelocity=0;
    double currentVelocity;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        drivetrain.setModeBlue();
        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("initialized");
    }

    @Override
    public void loop() {
        distance= drivetrain.getDistanceFromGoal();
        currentVelocity=shoot1.getVelocity();
        if (gamepad1.a){
            shootingVelocity=1600;
        }
        else if (gamepad1.b){
            shootingVelocity=1700;
        }
        else if (gamepad1.x){
            shootingVelocity=1800;
        }
        else if (gamepad1.y){
            shootingVelocity=1900;
        }

        if(gamepad1.dpad_up){
            shootingVelocity=shootingVelocity+100;
        }
        else if(gamepad1.dpad_down){
            shootingVelocity=shootingVelocity-100;
        }

        if (gamepad1.right_bumper){
            shoot1.setVelocity(shootingVelocity);
            shoot2.setVelocity(-shootingVelocity);
        }else{
            shoot1.setVelocity(0);
            shoot2.setVelocity(0);
        }

        telemetry.addData("distance",distance);
        telemetry.addData("shootingVelocity",shootingVelocity);
        telemetry.addData("currentVelocity",currentVelocity);
        telemetry.update();
    }
}
