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
        // pidTimer.reset();

    }

    /*public ServoRotate(HardwareMap hardwareMap, String servoName, String servoName2) {
        this(hardwareMap, servoName, servoName2);
    }*/
    public double getPosition(){
        return servo.getPosition();
    }
    public void startRotate(double position, double angle, double startAngle) {
        //double position2 = 1 - position;
        startAngle = startAngle / 360;
        if (position >= .39) {
            position = 0;
            //position2 = 1;
        }
        //else {
        position += (angle / 360) * .4;
       // position2 -= (angle / 360) * .4;
        //}
        if (position >= .39){
            position = .4;
            //position2 = .6;
        }
        servo.setPosition(position + startAngle);
        servo2.setPosition(position + startAngle);
    }
}