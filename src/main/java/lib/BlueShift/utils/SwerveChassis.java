package lib.BlueShift.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveChassis {
    /**
     * Converts the size of the robot to the positions of the swerve modules (4 modules)
     * <p>
     * Refer to this link for more information: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-drive-kinematics
     * @param trackWidth
     * @param wheelBase
     * @return Translation2d[] of the module positions
     */
    public static Translation2d[] sizeToModulePositions(double trackWidth, double wheelBase) {
        return new Translation2d[] {
            // Front left
            new Translation2d(wheelBase / 2, trackWidth / 2),
            // Front right
            new Translation2d(wheelBase / 2, -trackWidth / 2),
            // Back left
            new Translation2d(-wheelBase / 2, trackWidth / 2),
            // Back right
            new Translation2d(-wheelBase / 2, -trackWidth / 2)
        };
    }
}