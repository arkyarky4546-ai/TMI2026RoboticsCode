package org.firstinspires.ftc.teamcode;
import static com.pedropathing.math.MathFunctions.normalizeAngle;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class ServoRotate {
    private Servo servo;
    private Servo servo2;
    // private ElapsedTime pidTimer = new ElapsedTime();

    public ServoRotate(HardwareMap hardwareMap, String servoName, String servoName2) {
        servo = hardwareMap.get(Servo.class, servoName);
        servo2 = hardwareMap.get(Servo.class, servoName2);
        servo.setPosition(0);
        servo2.setPosition(0);
        // pidTimer.reset();

    }

    /*public ServoRotate(HardwareMap hardwareMap, String servoName, String servoName2) {
        this(hardwareMap, servoName, servoName2);
    }*/
    public double getPosition(){
        return servo.getPosition();
    }
    public void startRotate(double position, double angle, double maxAngle) {
        //double position2 = 1 - position;
        maxAngle = maxAngle/355 * 2/5;
        if (position >= maxAngle - .01) {
            position = maxAngle - .4 * 360/355;
            //position2 = 1;
        }
        //else {
        position += (angle / 355) * maxAngle;
       // position2 -= (angle / 360) * .4;
        //}
        if (position >= maxAngle - .01){
            position = maxAngle;
            //position2 = .6;
        }
        if ( position == 0){
            position = (maxAngle - .4 * 360 / 355);
        }

        servo.setPosition(position);
        servo2.setPosition(position);
    }
}