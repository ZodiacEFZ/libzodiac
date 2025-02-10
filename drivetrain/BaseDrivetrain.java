package frc.libzodiac.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface BaseDrivetrain extends Sendable {
    PoseEstimator<?> getPoseEstimator();

    Pigeon2 getGyro();

    Pose2d getPose();

    void setPose(Pose2d pose2d);

    ChassisSpeeds getRobotRelativeSpeeds();

    void driveRobotRelative(ChassisSpeeds chassisSpeeds);

    Subsystem getSubsystemBase();

    PathFollowingController getPathFollowingController();

    Field2d getField();

    double getMaxAngularVelocity();

    SwerveModuleState[] getModuleStates();
}
