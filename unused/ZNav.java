package frc.libzodiac.unused;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public final class ZNav extends SubsystemBase {

    private final Optional<DifferentialDrivePoseEstimator> diff_odo = Optional.empty();
    private final Optional<SwerveDrivePoseEstimator> swerve_odo = Optional.empty();

    public interface Gyro {
        double yaw();
    }

    public interface DiffOdometer {
        DifferentialDriveWheelPositions curr();
    }

    public interface SwerveOdometer {
        SwerveModulePosition[] curr();
    }

    public record Estimate(Pose2d pose, double timestamp) {
        public Pose2d pose() {
            return this.pose;
        }

        public double timestamp() {
            return this.timestamp;
        }
    }
}
