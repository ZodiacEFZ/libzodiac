package frc.libzodiac.api;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The drivetrain interface
 */
public interface Drivetrain extends Subsystem {
    /**
     * Get the pose estimator for the drivetrain
     *
     * @return the pose estimator
     */
    PoseEstimator<?> getPoseEstimator();

    /**
     * Get the gyro for the drivetrain
     *
     * @return the gyro
     */
    Pigeon2 getGyro();

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
    ChassisSpeeds getRobotRelativeSpeeds();

    /**
     * Drive the robot at the specified speeds
     *
     * @param chassisSpeeds the speeds to drive the robot at
     */
    void driveRobotRelative(ChassisSpeeds chassisSpeeds);

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
    SwerveModuleState[] getModuleStates();

    /**
     * Get whether the drivetrain is swerve or not
     *
     * @return whether the drivetrain is swerve or not
     */
    boolean isSwerve();

    /**
     * Brake the drivetrain
     */
    void brake();
}
