package frc.libzodiac.hardware;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.Units;
import frc.libzodiac.api.Drivetrain;
import frc.libzodiac.hardware.limelight.LimelightHelpers;

import java.util.ArrayList;

public class Limelight {
    private static final ArrayList<Limelight> limelights = new ArrayList<>();
    private final String name;
    private final Pigeon2 gyro;
    private final PoseEstimator<?> poseEstimator;

    public Limelight(Drivetrain drivetrain) {
        this("limelight", drivetrain);
    }

    public Limelight(String name, Drivetrain drivetrain) {
        this.name = name;
        this.gyro = drivetrain.getGyro();
        this.poseEstimator = drivetrain.getPoseEstimator();

        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port + limelights.size() * 10, this.name + ".local", port);
        }

        limelights.add(this);
    }

    public static void update() {
        for (Limelight limelight : limelights) {
            limelight.updateOdometry();
        }
    }

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

    public void setValidIDs(int[] validIDs) {
        LimelightHelpers.SetFiducialIDFiltersOverride(this.name, validIDs);
    }

    public double getTx() {
        return LimelightHelpers.getTX(this.name);
    }

    public double getTy() {
        return LimelightHelpers.getTY(this.name);
    }

    public double getTa() {
        return LimelightHelpers.getTA(this.name);
    }

    public double getAprilTagId() {
        return LimelightHelpers.getFiducialID(this.name);
    }
}
