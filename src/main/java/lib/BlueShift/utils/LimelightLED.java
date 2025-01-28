package lib.BlueShift.utils;

import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightLED {
    /**
     * Blink the LEDs on the limelight for a given duration
     * @param limelightName
     */
    public static final void blinkLeds(String limelightName) {
        blinkLeds(limelightName, 1);
    }

    /**
     * Blink the LEDs on the limelight for a given duration
     * @param limelightName
     * @param duration
     */
    public static final void blinkLeds(String limelightName, double duration) {
        new Thread(() -> {
            try {
                NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ledMode").setDouble(2);
                Thread.sleep((int)duration * 1000);
                NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ledMode").setDouble(0);
            } catch(Exception e) {}
        }).start();
    }
}
