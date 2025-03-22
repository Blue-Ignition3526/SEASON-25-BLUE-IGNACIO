// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

public class AllianceFlipUtil {
  public static double applyX(double x, boolean force) {
    return force || shouldFlip() ? FieldConstants.kFieldLength - x : x;
  }

  public static double applyY(double y, boolean force) {
    return force || shouldFlip() ? FieldConstants.kFieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation, boolean force) {
    return new Translation2d(applyX(translation.getX(), force), applyY(translation.getY(), force));
  }

  public static Rotation2d apply(Rotation2d rotation, boolean force) {
    return force || shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose, boolean force) {
    return force || shouldFlip()
        ? new Pose2d(apply(pose.getTranslation(), force), apply(pose.getRotation(), force))
        : pose;
  }

  public static Translation3d apply(Translation3d translation, boolean force) {
    return new Translation3d(
        applyX(translation.getX(), force), applyY(translation.getY(), force), translation.getZ());
  }

  public static Rotation3d apply(Rotation3d rotation, boolean force) {
    return force || shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static Pose3d apply(Pose3d pose, boolean force) {
    return new Pose3d(apply(pose.getTranslation(), force), apply(pose.getRotation(), force));
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}