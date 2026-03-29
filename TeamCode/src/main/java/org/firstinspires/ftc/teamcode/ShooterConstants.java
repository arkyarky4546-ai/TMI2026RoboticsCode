package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.bylazar.configurables.annotations.Configurable;

public class ShooterConstants {
    public static Pose GOAL_POSE_RED = new Pose(138,-138);
    public static Pose GOAL_POSE_BLUE = new Pose(138,-6);
    public static double Score_Height = 32; //measured from the flywheel not the turret, but it should work for either
    public static double Score_Angle = Math.toRadians(-20);
   // public static double Score_Angle = Math.toRadians(-30);
    public static double Pass_Through_Radius = 5;
    private static double flyOff = 0.0;
    private static double flyMin = 0.0;
    private static double flyMax = 2000.0;
    public static double Hood_Max_Angle = 2.517;
    public static double Hood_Min_Angle = .25;//from like 0 to pi
    public static double hoodDegreesToServoTicks(double radians){ //turn calculated velocity to motor commands

        //return 0.0226 * degrees -.7443;
        //return 0.3*radians - 0.8;
        //return -1.72*radians + 1.9; //our brains are
        return -1.18*radians + 1.17; //our brains are
    }
    public static double getFlyWheelTicksFromVelocity(double velocity){
        //return MathFunctions.clamp(94.501 * velocity/12 - 187.96 + flyOff, flyMin, flyMax);
        //return MathFunctions.clamp(-Math.sqrt(-4201.68067227*velocity + 1162629.75779) + 1470.58823529, flyMin, flyMax);
        return MathFunctions.clamp(5.81395348837*(velocity-49.1) + 200, flyMin, flyMax);
    }
}