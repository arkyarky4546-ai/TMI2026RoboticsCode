package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AxonRotator {

    private CRServo servo;
    private CRServo servo2;
    private AnalogInput encoder;

    private double encoderMaxVoltage = 3.3;

    private double targetAngle = 0.0;

    private static final double kP = 0.0038;
    private static final double kD = 0.00012;
    private static final double kI = 0.0001;

    private static final double HOLD_DEADBAND = 0.05;
    private static final double MAX_POWER = 0.3;

    private static final double GEAR_RATIO_FACTOR = 20.0 / 50.0;

    private boolean encoderInverted = false;

    private double lastError = 0.0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private double absoluteCurrent = 0.0;

    public AxonRotator(HardwareMap hardwareMap, String servoName, String servoName2, String encoderName, boolean reverseEncoder, double voltageScale) {
        servo = hardwareMap.get(CRServo.class, servoName);
        servo2 = hardwareMap.get(CRServo.class, servoName2);
        encoder = hardwareMap.get(AnalogInput.class, encoderName);
        this.encoderInverted = reverseEncoder;
        this.encoderMaxVoltage = voltageScale;

        targetAngle = getAbsoluteAngle();

        pidTimer.reset();
    }

    public AxonRotator(HardwareMap hardwareMap, String servoName, String servoName2, String encoderName, boolean reverseEncoder) {
        this(hardwareMap, servoName, servoName2, encoderName, reverseEncoder, 3.3);
    }

    public void startRotate(double degrees) {
        if (degrees >= 0) degrees = 120;
        else if (degrees < 0) degrees = -120;
        else degrees = 0;
        double current = absoluteCurrent;
        double adjustedDegrees = degrees * GEAR_RATIO_FACTOR;
        double newTarget = current + adjustedDegrees;
        absoluteCurrent = newTarget;

        targetAngle = normalizeAngle(newTarget);
    }

    public void update(Telemetry telemetry) {
        double currentAngle = getAbsoluteAngle();
        double error = shortestPathError(targetAngle, currentAngle);
        telemetry.addData("spindex error: ", error);
        double currentTime = pidTimer.seconds();
        double derivative = (error - lastError) / currentTime;
        pidTimer.reset();
        lastError = error;
        double power = 0;
        if (Math.abs(error) > HOLD_DEADBAND) {
            power = (error * kP) + (derivative * kD);
        } else {
            power = 0;
            lastError = error;
        }
        double clippedPower = Range.clip(power, -MAX_POWER, MAX_POWER);
        telemetry.addData("turretPower: ", clippedPower);
        servo.setPower(clippedPower);
        servo2.setPower(clippedPower);
    }

    public double getAbsoluteAngle() {
        double voltage = Math.max(0, encoder.getVoltage());
        double angle = (voltage / encoderMaxVoltage) * 360.0;

        if (encoderInverted) {
            angle = 360.0 - angle;
        }

        return normalizeAngle(angle);
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360.0;
       // if (angle < 0) angle += 360.0;
        return Range.clip(angle, 0, 360);
    }

    private double shortestPathError(double target, double current) {
        double error = target - current;

        while (error > 180.0)  error -= 360.0;
        while (error <= -180.0) error += 360.0;

        return error;
    }

    public double getCurrentPosition() { return getAbsoluteAngle(); }
    public double getTargetPosition() { return targetAngle; }
}