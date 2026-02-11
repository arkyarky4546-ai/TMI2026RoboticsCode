package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
    private static double CENTER_POSITION = 0.924;


    private double targetX = 0.0;
    private double targetY = 0.0;

    private double currentServoPosition = CENTER_POSITION;
    private final double gearRatio =  1.0055911*1.11111111;//input / output 1.0055911

    private boolean autoAiming;
    private Timer lastManual = new Timer();


    public AutoTurret(HardwareMap hardwareMap, String leftServoName, String rightServoName) {
        servoLeft = hardwareMap.get(Servo.class, leftServoName);
        servoRight = hardwareMap.get(Servo.class, rightServoName);
        autoAiming = true;
    }

    public void setTargetCoordinates(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public void updateAuto(Follower follower, Telemetry telemetry, double Angle) {
        double servoAngleNeeded = Angle * gearRatio/356; //360
        double targetPos = (servoAngleNeeded) + (1 - CENTER_POSITION);

        if(targetPos < 0){
            targetPos += 360/355*1.111111111;

        }
        targetPos = Range.clip(targetPos, 0.0, 1.0);
        servoLeft.setPosition(1 - targetPos);
        servoRight.setPosition(1 - targetPos);
        currentServoPosition = targetPos;
    }


    public void setModeRed(){
        targetX = 138.0;
        targetY = -138.0;
        CENTER_POSITION = .63;
    }

    public void setModeBlue(){
        targetX = 138.0;
        targetY = 0.0;
        CENTER_POSITION = .89;
    }

}