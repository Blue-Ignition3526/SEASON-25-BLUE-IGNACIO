package frc.robot.speedAlterators;

import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Ranger;
import lib.BlueShift.control.SpeedAlterator;

public class AlignToBranchWithRanger extends SpeedAlterator {
    private final Ranger ranger;
    private final Supplier<Boolean> isRightSupplier;

    public AlignToBranchWithRanger(Ranger ranger, Supplier<Boolean> isRightSupplier) {
        this.ranger = ranger;
        this.isRightSupplier = isRightSupplier;
    }

    @Override
    public void onEnable() {
        // TODO: RESET PID
    }

    @Override
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        return speeds;
    }
}
