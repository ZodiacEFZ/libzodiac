package frc.libzodiac.api;

import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Optional;

/**
 * The drivetrain interface
 */
public interface Drivetrain extends Subsystem {
    /**
     * Get the current pose of the robot
     *
     * @return the current pose of the robot
     */
    Pose2d getPose();

    /**
     * Set the robot to a specific pose
     *
     * @param pose2d the pose to set the robot to
     */
    void setPose(Pose2d pose2d);

    /**
     * Get the current speeds of the robot
     *
     * @return the current speeds of the robot
     */
    ChassisSpeeds getRobotCentricSpeeds();

    /**
     * Drive the robot at the specified speeds
     *
     * @param chassisSpeeds the speeds to drive the robot at
     */
    void driveRobotCentric(ChassisSpeeds chassisSpeeds);

    /**
     * Get the path following controller for the drivetrain
     *
     * @return the path following controller
     */
    PathFollowingController getPathFollowingController();

    /**
     * Get the field
     *
     * @return the field
     */
    Field2d getField();

    /**
     * Get the maximum turning speed of the robot
     *
     * @return the maximum turning speed of the robot
     */
    double getMaxAngularVelocity();

    /**
     * Get the module states of the robot
     *
     * @return the module states of the robot
     */
    Optional<SwerveModuleState[]> getModuleStates();

    void shutdown();

    /**
     * Brake the drivetrain
     */
    void brake();

    AngularVelocity getAngularVelocity();

    void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
                              Matrix<N3, N1> visionMeasurementStdDevs);
}
