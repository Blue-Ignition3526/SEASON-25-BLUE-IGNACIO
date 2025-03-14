package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    public static final AprilTagFieldLayout kApriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final Translation2d kFieldCenter = new Translation2d(kApriltagFieldLayout.getFieldLength() / 2, kApriltagFieldLayout.getFieldWidth() / 2);
}
