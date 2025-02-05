package frc.libzodiac.hardware;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.units.Units;
import frc.libzodiac.drivetrain.ZDrivetrain;
import frc.libzodiac.hardware.limelight.LimelightHelpers;

public class Limelight {
    // TODO
    // Currently multiple limelights are not supported, use one limelight only
    private static Limelight instance = null;
    private final PoseEstimator<?> poseEstimator;
    private final Pigeon2 gyro;

    private Limelight(ZDrivetrain drivetrain) {
        this.gyro = drivetrain.getGyro();
        this.poseEstimator = drivetrain.getPoseEstimator();
    }

    public static Limelight get() {
        return instance;
    }

    public static void setValidIDs(int[] validIDs) {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
    }

    public static void update() {
        if (instance != null) {
            instance.updateOdometry();
        }
    }

    private void updateOdometry() {
        LimelightHelpers.SetRobotOrientation("limelight",
                this.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        // if our angular velocity is greater than 2 rotations per second, ignore vision updates
        if (Math.abs(this.gyro.getAngularVelocityZWorld().getValue()
                .in(Units.RadiansPerSecond)) > 4 * Math.PI || poseEstimate.tagCount == 0) {
            // TODO: Maybe increase the trust when we get closer
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
        }
    }

    public static void init(ZDrivetrain drivetrain) {
        instance = new Limelight(drivetrain);
    }
}
