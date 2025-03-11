package frc.libzodiac.hardware;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.Units;
import frc.libzodiac.api.Drivetrain;
import frc.libzodiac.api.Gyro;
import frc.libzodiac.hardware.limelight.LimelightHelpers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.stream.IntStream;

/**
 * A class to interface with the Limelight camera.
 */
public class Limelight {
    /**
     * The list of all Limelights.
     */
    private static final ArrayList<Limelight> LIMELIGHTS = new ArrayList<>();
    /**
     * The name of the Limelight.
     */
    private final String name;
    /**
     * The gyro of the drivetrain.
     */
    private final Gyro gyro;
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

        IntStream.rangeClosed(5800 + LIMELIGHTS.size() * 10, 5809 + LIMELIGHTS.size() * 10)
                 .forEachOrdered(port -> PortForwarder.add(port, this.name + ".local", port));

        LIMELIGHTS.add(this);
    }

    /**
     * Update the odometry with all Limelights.
     */
    public static void update() {
        for (Limelight limelight : LIMELIGHTS) {
            limelight.updateOdometry();
        }
    }

    /**
     * Updates the odometry.
     */
    private void updateOdometry() {
        LimelightHelpers.SetRobotOrientation(this.name, this.poseEstimator.getEstimatedPosition()
                                                                          .getRotation()
                                                                          .getDegrees(), 0, 0, 0, 0,
                                             0);
        final var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.name);

        // FIXME: poseEstimate == null
        if (poseEstimate == null) {
            return;
        }

        // if our angular velocity is greater than 2 rotations per second, ignore vision updates
        if (Math.abs(this.gyro.getYawAngularVelocity().in(Units.RadiansPerSecond)) >= Math.PI) {
            return;
        }

        if (poseEstimate.tagCount <= 0) {
            return;
        }

        final var minDistance = Arrays.stream(poseEstimate.rawFiducials)
                                      .map(x -> x.distToCamera)
                                      .min(Comparator.naturalOrder())
                                      .orElseThrow();
        var visionMeasurementStdDevs = VecBuilder.fill(.7, .7, 9999999);
        if (minDistance < 1) {
            visionMeasurementStdDevs = VecBuilder.fill(.3, .3, 9999999);
        } else if (minDistance < 2) {
            visionMeasurementStdDevs = VecBuilder.fill(.5, .5, 9999999);
        }
        this.poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds,
                                                visionMeasurementStdDevs);

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
    public double getTX() {
        return LimelightHelpers.getTX(this.name);
    }

    /**
     * Gets the vertical offset from the crosshair to the target in degrees.
     *
     * @return Vertical offset angle in degrees.
     */
    public double getTY() {
        return LimelightHelpers.getTY(this.name);
    }

    /**
     * Gets the target area as a percentage of the image (0-100%).
     *
     * @return Target area percentage (0-100).
     */
    public double getTA() {
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

    public void setPipeline(int index) {
        LimelightHelpers.setPipelineIndex(this.name, index);
    }
}
