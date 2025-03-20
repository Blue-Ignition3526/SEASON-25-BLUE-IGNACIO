package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final AprilTagFieldLayout kApriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final double kFieldWidth = kApriltagFieldLayout.getFieldWidth();
    public static final double kFieldLength = kApriltagFieldLayout.getFieldLength();
    public static final Translation2d kFieldCenter = new Translation2d(kFieldLength / 2, kFieldWidth / 2);

    // ! FROM MECHANICAL ADVANTAGE
    public static class CoralStation {
        public static final double stationLength = Units.inchesToMeters(79.750);
        public static final Pose2d rightCenterFace =
            new Pose2d(
                Units.inchesToMeters(33.526),
                Units.inchesToMeters(25.824),
                Rotation2d.fromDegrees(144.011 - 90));
        public static final Pose2d leftCenterFace =
            new Pose2d(
                rightCenterFace.getX(),
                kFieldWidth - rightCenterFace.getY(),
                Rotation2d.fromRadians(-rightCenterFace.getRotation().getRadians()));
  }

  // ! FROM MECHANICAL ADVANTAGE
  public enum ReefLevel {
    L1(Units.inchesToMeters(25.0), 0),
    L2(Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L3(Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L4(Units.inchesToMeters(72), -90);

    ReefLevel(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // Degrees
    }

    public static ReefLevel fromLevel(int level) {
        switch (level) {
            case 1: return L1;
            case 2: return L2;
            case 3: return L3;
            case 4: return L4;
        
            default:
                return L4;
        }
    }

    public final double height;
    public final double pitch;
  }

  // ! FROM MECHANICAL ADVANTAGE
  public static class Reef {
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final Translation2d center = new Translation2d(Units.inchesToMeters(176.746), kFieldWidth / 2.0);
    public static final double faceToZoneLine = Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] faceCenters = new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final Pose2d[] thresholdingFaceCenters = new Pose2d[6]; // Starting facing the driver station in clockwise order 
                                                                          // * Faces offset to the zone line marked with tape
    public static final List<Map<ReefLevel, Pose3d>> branchPositions = new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
    public static final List<Map<ReefLevel, Pose2d>> branchPositions2d = new ArrayList<>();

    static {
      // Initialize faces
      faceCenters[0] = kApriltagFieldLayout.getTagPose(18).get().toPose2d();
      faceCenters[1] = kApriltagFieldLayout.getTagPose(19).get().toPose2d();
      faceCenters[2] = kApriltagFieldLayout.getTagPose(20).get().toPose2d();
      faceCenters[3] = kApriltagFieldLayout.getTagPose(21).get().toPose2d();
      faceCenters[4] = kApriltagFieldLayout.getTagPose(22).get().toPose2d();
      faceCenters[5] = kApriltagFieldLayout.getTagPose(17).get().toPose2d();

      // Initialize thresholding face poses
      for (int face = 0; face < 6; face++) {
        
      }

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
        Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
        Map<ReefLevel, Pose2d> fillRight2d = new HashMap<>();
        Map<ReefLevel, Pose2d> fillLeft2d = new HashMap<>();
        for (var level : ReefLevel.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          var rightBranchPose =
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians()));
          var leftBranchPose =
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians()));

          fillRight.put(level, rightBranchPose);
          fillLeft.put(level, leftBranchPose);
          fillRight2d.put(level, rightBranchPose.toPose2d());
          fillLeft2d.put(level, leftBranchPose.toPose2d());
        }
        branchPositions.add(fillRight);
        branchPositions.add(fillLeft);
        branchPositions2d.add(fillRight2d);
        branchPositions2d.add(fillLeft2d);
      }
    }

    static {
        System.out.println();
    }
  }
}