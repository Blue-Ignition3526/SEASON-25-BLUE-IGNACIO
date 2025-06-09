package lib.BlueShift.odometry.vision.camera;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.QuestNav;
import lib.BlueShift.odometry.vision.OdometryCamera;
import lib.BlueShift.odometry.vision.VisionOdometryPoseEstimate;

public class QuestNavOdometryCamera extends SubsystemBase implements OdometryCamera {
    private QuestNav quest;
    private Transform2d questToBot;
    private boolean m_enabled;
    private double lastTimestamp = -1;

    public QuestNavOdometryCamera(QuestNav quest, Transform2d questToBot, boolean enabled) {
        // Store variables
        this.quest = quest;
        this.questToBot = questToBot;
        this.m_enabled = enabled;
    }

    @Override
    public String getCameraName() {
        return getName();
    }

    @Override
    public boolean isEnabled() {
        return m_enabled;
    }

    @Override
    public void setEnabled(boolean enabled) {
        m_enabled = enabled;
    }

    @Override
    public void enable() {
        m_enabled = true;
    }

    @Override
    public void disable() {
        m_enabled = false;
    }

    @Override
    public Optional<VisionOdometryPoseEstimate> getEstimate() {
        if (!m_enabled || !quest.getConnected() || !quest.getTrackingStatus()) return Optional.empty();
        lastTimestamp = quest.getTimestamp();
        return Optional.of(new VisionOdometryPoseEstimate(
            quest.getPose().transformBy(questToBot.inverse()), 
            lastTimestamp,
            -1,
            -1
        ));
    }

    @Override
    public double getLastTimestamp() {
        return lastTimestamp;
    }

    @Override
    public void periodic() {
        quest.cleanupResponses();
        quest.processHeartbeat();
    }
}
