package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AxonRotator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class autoShootTest extends OpMode {
    Servo turretRight; //launchservo
    Servo turretLeft; //launchservo
    CRServo artifactSpinner;
    Servo artifactPush;
    DcMotorEx shoot1;
    DcMotorEx shoot2;
    DcMotor intakeMotor;
    DcMotor intakeMotor1;
    boolean index12 = false;
    Servo intakeGate;
    Servo shooterHood;
    AxonRotator spindexRotor;
    CRServo smart;
    double kickZero = 0.85;
    double kickUp = 0.75;
    double TargetVelocity = 1400;
    Servo wally;
    double turTurn = .48;
    private Limelight3A limelight3A;
    double shooterHoodPos = 0.5;
    double shootVelocity = 1400;
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");
        intakeGate = hardwareMap.get(Servo.class, "intakeGate");

        //artifact storage
        // artifactSpinner = hardwareMap.get(CRServo.class, "spindexRoter");
        artifactPush = hardwareMap.get(Servo.class, "push");

        //shooter
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turretRight.setPosition(turTurn);
        turretLeft.setPosition(turTurn);
        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        wally = hardwareMap.get(Servo.class, "wally");
        // smart = hardwareMap.get(CRServo.class, "spindexRoter");
        spindexRotor = new AxonRotator(hardwareMap,"spindexRoter","slave", "smartTrack", true);
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterHood = hardwareMap.get(Servo.class, "shooterHood");
        shooterHood.setPosition(0.62);
        artifactPush.setPosition(kickZero);
    }

    @Override
    public void loop() {
        shooterHood.setPosition(shooterHoodPos);
        if (gamepad1.dpad_up) {
            shooterHoodPos += 0.05;
        }
        if (gamepad1.dpad_down) {
            shooterHoodPos -= 0.05;
        }
        if (gamepad1.y) {
            shootVelocity += 500;
        }
        if (gamepad1.y) {
            shootVelocity -= 500;
        }
        telemetry.addData("shooterHoodPos: ", shooterHoodPos);
        telemetry.addData("shootVelocity: ", shootVelocity);
        telemetry.update();

    }
}
