package lib.BlueShift.control;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class SpeedAlterator {
    /**
     * Provided the desired input speeds, modify it to the desired output speeds
     * @param speeds
     * @param robotRelative
     * @return the altered speeds
     */
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        return speeds;
    };

    /**
     * Called when the SpeedAlterator is enabled
     */
    public void onEnable() {};

    /**
     * Called when the SpeedAlterator is disabled
     */
    public void onDisable() {};
}
