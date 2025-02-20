package frc.libzodiac.hardware;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.Units;
import frc.libzodiac.api.Drivetrain;
import frc.libzodiac.hardware.limelight.LimelightHelpers;

import java.util.ArrayList;

/**
 * A class to interface with the Limelight camera.
 */
public class Limelight {
    /**
     * The list of all Limelights.
     */
    private static final ArrayList<Limelight> limelights = new ArrayList<>();
    /**
     * The name of the Limelight.
     */
    private final String name;
    /**
     * The gyro of the drivetrain.
     */
    private final Pigeon2 gyro;
    /**
     * The pose estimator of the drivetrain.
     */
    private final PoseEstimator<?> poseEstimator;

    /**
     * Construct a new Limelight.
     *
     * @param drivetrain The drivetrain.
     */
    public Limelight(Drivetrain drivetrain) {
        this("limelight", drivetrain);
    }

    /**
     * Construct a new Limelight.
     *
     * @param name       The name of the Limelight.
     * @param drivetrain The drivetrain.
     */
    public Limelight(String name, Drivetrain drivetrain) {
        this.name = name;
        this.gyro = drivetrain.getGyro();
        this.poseEstimator = drivetrain.getPoseEstimator();

        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port + limelights.size() * 10, this.name + ".local", port);
        }

        limelights.add(this);
    }

    /**
     * Update the odometry with all Limelights.
     */
    public static void update() {
        for (Limelight limelight : limelights) {
            limelight.updateOdometry();
        }
    }

    /**
     * Updates the odometry.
     */
    private void updateOdometry() {
        LimelightHelpers.SetRobotOrientation(this.name,
                this.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.name);

        // if our angular velocity is greater than 2 rotations per second, ignore vision updates
        if (poseEstimate.tagCount > 0 && Math.abs(
                this.gyro.getAngularVelocityZWorld().getValue().in(Units.RadiansPerSecond)) < Math.PI) {
            // TODO: Maybe increase the trust when we get closer
            this.poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            this.poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
        }
    }

    /**
     * Sets th valid IDs of the Limelight.
     *
     * @param validIDs The valid IDs.
     */
    public void setValidIDs(int[] validIDs) {
        LimelightHelpers.SetFiducialIDFiltersOverride(this.name, validIDs);
    }

    /**
     * Gets the horizontal offset from the crosshair to the target in degrees.
     *
     * @return Horizontal offset angle in degrees.
     */
    public double getTx() {
        return LimelightHelpers.getTX(this.name);
    }

    /**
     * Gets the vertical offset from the crosshair to the target in degrees.
     *
     * @return Vertical offset angle in degrees.
     */
    public double getTy() {
        return LimelightHelpers.getTY(this.name);
    }

    /**
     * Gets the target area as a percentage of the image (0-100%).
     *
     * @return Target area percentage (0-100).
     */
    public double getTa() {
        return LimelightHelpers.getTA(this.name);
    }

    /**
     * Gets the AprilTag ID.
     *
     * @return The AprilTag ID.
     */
    public double getAprilTagId() {
        return LimelightHelpers.getFiducialID(this.name);
    }
}
