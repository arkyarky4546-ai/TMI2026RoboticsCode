package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorThread extends Thread {
    private volatile boolean running = true;

    private DistanceSensor disBL;
    private DistanceSensor disST;

    // volatile ensures the main thread always reads the most recent value from memory
    private volatile boolean blHasBall = false;
    private volatile boolean stHasBall = false;

    // Constructor passes the mapped sensors into the thread
    public SensorThread(DistanceSensor disBL, DistanceSensor disST) {
        this.disBL = disBL;
        this.disST = disST;
    }

    @Override
    public void run() {
        while (running) {
            // Read distance and calculate the threshold in the background
            if (disBL != null) {
                blHasBall = disBL.getDistance(DistanceUnit.CM) < 4.0;
            }
            if (disST != null) {
                stHasBall = disST.getDistance(DistanceUnit.CM) < 4.0;
            }

            try {
                // 20ms sleep is the sweet spot for REV Hub I2C sensors.
                // Any faster and you're just reading stale cached data.
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    // Main thread calls these instant getter methods
    public boolean hasBallBL() { return blHasBall; }
    public boolean hasBallST() { return stHasBall; }

    public void stopThread() {
        running = false;
    }
}