package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.bylazar.configurables.annotations.Configurable;

public class ShooterConstants {
    public static Pose GOAL_POSE_RED = new Pose(138,-138);
    public static Pose GOAL_POSE_BLUE = new Pose(138,-6);
    public static double Score_Height = 26;
    public static double Score_Angle = Math.toRadians(-30);
    public static double Pass_Through_Radius = 5;
    private static double flyOff = 0.0;
    private static double flyMin = 0.0;
    private static double flyMax = 2000.0;
    public static double Hood_Max_Angle = 2.517;
    public static double Hood_Min_Angle = .25;//from like 0 to pi
    public static double hoodDegreesToServoTicks(double degrees){ //turn calculated velocity to motor commands
        return 0.0226 * degrees -.7443;
    }
    public static double getFlyWheelTicksFromVelocity(double velocity){
        return MathFunctions.clamp(94.501 * velocity/12 - 187.96 + flyOff, flyMin, flyMax);
    }
}
