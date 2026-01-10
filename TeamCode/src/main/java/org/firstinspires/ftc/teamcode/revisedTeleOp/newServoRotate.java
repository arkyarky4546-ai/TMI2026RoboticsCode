package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class newServoRotate {
    private Servo servo;
    private Servo servo2;
    private double currentAngle = 0.0;
    private int currentPos = 0;
    private final double gearRatio = 2/5;
    private final double conversionRatio = 355/355;
    private final double maxAngle = 360;
    private boolean shootOffset = false;

    public newServoRotate(HardwareMap hardwareMap, String servoName, String servoName2) {
        servo = hardwareMap.get(Servo.class, servoName);
        servo2 = hardwareMap.get(Servo.class, servoName2);
        servo.setPosition(0);
        servo2.setPosition(0);
    }
    public double getPosition(){
        return servo.getPosition();
    }
    public void offsetForShooting(boolean toBeOffset){
        double targetAngle;
        if(toBeOffset){
            shootOffset = true;
            targetAngle = currentAngle + 60;
        }
        else{
            shootOffset = false;
            targetAngle = currentAngle - 60;
        }
        if(targetAngle < 0){
            targetAngle += 360;
        }
        else if(targetAngle > 360){
            targetAngle -= 360;
        }
        double targetPos = targetAngle * gearRatio * conversionRatio;
        servo.setPosition(targetPos);
        servo2.setPosition(targetPos);
        currentAngle = targetAngle;

    }
    public void rotatePos(){
        double targetAngle = currentAngle + 120;
        if(targetAngle > 360) {
            targetAngle -= 360;
        }
        double targetPos = targetAngle * gearRatio * conversionRatio;
        servo.setPosition(targetPos);
        servo2.setPosition(targetPos);
        currentPos++;
        currentAngle = targetAngle;

    }
}
