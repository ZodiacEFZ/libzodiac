package frc.libzodiac;

import java.util.Optional;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ZNav extends SubsystemBase {

    private Optional<DifferentialDrivePoseEstimator> diff_odo = Optional.empty();
    private Optional<SwerveDrivePoseEstimator> swerve_odo = Optional.empty();

    public static interface Gyro {
        double yaw();
    }

    public static final record Estimate(Pose2d pose, double timestamp) {
        public Pose2d pose() {
            return this.pose;
        }

        public double timestamp() {
            return this.timestamp;
        }
    }

    public static interface DiffOdometer {
        DifferentialDriveWheelPositions curr();
    }

    public static interface SwerveOdometer {
        SwerveModulePosition[] curr();
    }
}
