package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Configurable
//@TeleOp
public class DumpTeleOp extends OpMode {
    /* newly created classes */
    Drivetrain drivetrain; //all driving functionality and pedro pathing

    private Servo servoLeft;
    private Servo servoRight;

    DcMotorEx intake1;
    DcMotorEx intake2;
    Servo intakeGate;

    Servo artifactPush;
    private final double kickZero = 0.85;
    private final double kickUp = 0.75;

    DcMotorEx shoot1;
    DcMotorEx shoot2;
    double intakePower=0.75;
    Servo wall;
    Servo shooterHood;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setModeRed();

        servoLeft = hardwareMap.get(Servo.class, "turretLeft");
        servoRight = hardwareMap.get(Servo.class, "turretRight");
        servoLeft.setPosition(0.96);
        servoRight.setPosition(0.96);

        intake1 = hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake1");
        intakeGate = hardwareMap.get(Servo.class, "intakeGate");
        intakeGate.setPosition(0);

        artifactPush = hardwareMap.get(Servo.class, "push");

        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        wall = hardwareMap.get(Servo.class, "wally");
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterHood = hardwareMap.get(Servo.class, "shooterHood");
        shooterHood.setPosition(0.4);
        artifactPush.setPosition(kickZero);


    }

    @Override
    public void loop() {
        double currentVelocity=shoot2.getVelocity();
        double targetVelocity=1200;
        drivetrain.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.xWasPressed(), gamepad1.bWasPressed());
        if(gamepad1.a){
            drivetrain.resetCurrentPose();
        }
        if(gamepad2.left_bumper){
            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            wall.setPosition(.5);
            shoot1.setVelocity(-0);
            shoot2.setVelocity(0);
        }
        else if(gamepad2.left_trigger>0.75){
            intake1.setPower(-intakePower);
            intake2.setPower(intakePower);
        }
        else{
            intake1.setPower(-0);
            intake2.setPower(0);
            shoot1.setVelocity(-0);
            shoot2.setVelocity(0);
        }
        if(gamepad2.right_bumper){
            shoot1.setVelocity(-targetVelocity);
            shoot2.setVelocity(targetVelocity);
            intake1.setPower(1);
            intake2.setPower(-1);
            if((shoot2.getVelocity() > (targetVelocity*.95))&&(shoot2.getVelocity()<targetVelocity*1.05)) {
                wall.setPosition(0);
                artifactPush.setPosition(kickUp);
            }
        }


    }
}
