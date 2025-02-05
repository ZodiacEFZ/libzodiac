package frc.libzodiac.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.util.sendable.Sendable;

public interface ZDrivetrain extends Sendable {
    PoseEstimator<?> getPoseEstimator();

    Pigeon2 getGyro();
}
