package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.revisedTeleOp.Drivetrain;

@Configurable
@TeleOp
public class shootingReg extends OpMode {
    org.firstinspires.ftc.teamcode.revisedTeleOp.Drivetrain drivetrain;

    DcMotorEx shoot1;
    DcMotorEx shoot2;

    DcMotorEx intake1;
    DcMotorEx intake2;

    private Follower follower;

    private final Pose startPose = new Pose(128,-26, Math.toRadians(47));

    double distance;
    double shootingVelocity=0;
    double currentVelocity;

    private static int isIntake = -1;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        intake1 = hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake1");
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("initialized");
    }

    @Override
    public void loop() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        distance= Math.sqrt((x - 144) * (x - 144) + (y + 0) * (y + 0));
        currentVelocity=shoot1.getVelocity();
        if (gamepad1.a){
            shootingVelocity=800;
        }
        else if (gamepad1.b){
            shootingVelocity=900;
        }
        else if (gamepad1.x){
            shootingVelocity=1000;
        }
        else if (gamepad1.y){
            shootingVelocity=1100;
        }

        if(gamepad1.dpad_up){
            shootingVelocity=1200;
        }
        else if(gamepad1.dpad_down){
            shootingVelocity=1300;
        }

        if (gamepad1.right_bumper){
            shoot1.setVelocity(-shootingVelocity);
            shoot2.setVelocity(shootingVelocity);
        }else{
            shoot1.setVelocity(0);
            shoot2.setVelocity(0);
        }
        follower.update();

        if (gamepad1.left_bumper) {
            isIntake = 1;
        }
        else if (gamepad1.dpad_right){
            isIntake = -1;
        }

        if (isIntake > 0) {
            intake1.setPower(1);
            intake2.setPower(-1);
        }


        telemetry.addData("x",x);
        telemetry.addData("y",y);
        telemetry.addData("distance",distance);
        telemetry.addData("shootingVelocity",shootingVelocity);
        telemetry.addData("currentVelocity",currentVelocity);
        telemetry.update();
    }
}
