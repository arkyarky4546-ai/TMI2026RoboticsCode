package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

//it's the same as the AutoAimTurret class but I added a safety in case the drivers want to aim manually
public class AutoTurret {

    private Servo servoLeft;
    private Servo servoRight;

    private static final double SERVO_RANGE_DEGREES = 361.0;
    private static double CENTER_POSITION = 0.73;


    private double targetX = 0.0;
    private double targetY = 0.0;

    private double currentServoPosition = CENTER_POSITION;
    private final double gearRatio =  1.0055911 * 23/21; //input / output 1.0055911

    private boolean autoAiming;
    private Timer lastManual = new Timer();

    private double targetPos;


    public AutoTurret(HardwareMap hardwareMap, String leftServoName, String rightServoName) {
        servoLeft = hardwareMap.get(Servo.class, leftServoName);
        //servoLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servoRight = hardwareMap.get(Servo.class, rightServoName);
        //servoRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servoLeft.setPosition(CENTER_POSITION);
        servoRight.setPosition(CENTER_POSITION);
        autoAiming = true;
    }

    public void setTargetCoordinates(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public double getTargetPos(){
        return targetPos;
    }

    public void updateAuto(Follower follower, Telemetry telemetry, double Angle, boolean aim) {
        if(aim) {
            double servoAngleNeeded = Angle * gearRatio / 360; //356
            //double targetPos = (servoAngleNeeded) + (1 - CENTER_POSITION);
            targetPos = (servoAngleNeeded) + CENTER_POSITION;

//            if (targetPos < 0) {
//                targetPos += 360 / 355;
//            }

            if (targetPos > 1) {
                targetPos -= 360 / 355;
            }
            targetPos = Range.clip(targetPos, 0.0, 1.0);
//            servoLeft.setPosition(1 - targetPos);
//            servoRight.setPosition(1 - targetPos);
            servoLeft.setPosition(targetPos);
            servoRight.setPosition(targetPos);
            currentServoPosition = targetPos;
        }
    }


    public void setModeRed(){
        targetX = 138.0;
        targetY = -138.0;
        //CENTER_POSITION = .63;
    }

    public void setModeBlue(){
        targetX = 138.0;
        targetY = 0.0;
        //CENTER_POSITION = .89;
    }
    public void manualLeft(){
        if((servoLeft.getPosition() > 0.15) && (lastManual.getElapsedTime() > 100)){
            double toBeSetPos = servoLeft.getPosition() - 0.01;
            servoLeft.setPosition(toBeSetPos);
            servoRight.setPosition(toBeSetPos);
            lastManual.resetTimer();
        }
    }

    public void manualRight(){
        if((servoLeft.getPosition() < 0.85) && (lastManual.getElapsedTime() > 100)){
            double toBeSetPos = servoLeft.getPosition() + 0.01;
            servoLeft.setPosition(toBeSetPos);
            servoRight.setPosition(toBeSetPos);
            lastManual.resetTimer();
        }
    }

    public void setCenter(){
        servoLeft.setPosition(CENTER_POSITION);
        servoRight.setPosition(CENTER_POSITION);

    }


}