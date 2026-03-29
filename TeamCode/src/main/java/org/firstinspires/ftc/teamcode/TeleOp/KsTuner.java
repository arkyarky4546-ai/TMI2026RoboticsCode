package org.firstinspires.ftc.teamcode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intakeShoot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp
public class KsTuner extends OpMode {
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    double[] increments = {0.000001, 0.00001, 0.0001, 0.001, 0.01};
    public static double kS = 0.155;
    public static double kV,kP;
    int incrementIdx = 4;

    @Override
    public void loop() {
        if(gamepad1.dpadRightWasPressed() && incrementIdx < 4){
            incrementIdx++;
        }
        if(gamepad1.dpadLeftWasPressed() && incrementIdx > 0){
            incrementIdx--;
        }
        double currentStep = increments[incrementIdx];
        if(gamepad1.yWasPressed()){kS +=currentStep;}
        if(gamepad1.aWasPressed()){kS -=currentStep;}
        shooter1.setPower(-kS);
        shooter2.setPower(kS);
        telemetry.addData("kS","%.6f", kS);
        telemetry.addData("kS", "%.6f",currentStep);
        telemetry.addData("ticks", shooter2.getVelocity());
        //telemetry.addData("wallPos", );

    }
    @Override
    public void init(){
        shooter1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void start(){

        //opmodeTimer.resetTimer();
    }
}




