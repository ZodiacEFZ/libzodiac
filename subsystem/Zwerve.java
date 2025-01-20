// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libzodiac.subsystem;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.group.TalonFXSwerveModule;

public class Zwerve extends SubsystemBase {
    // Distance between centers of right and left wheels on robot
    public final double ROBOT_WIDTH;
    // Distance between front and back wheels on robot
    public final double ROBOT_LENGTH;
    public final double MAX_SPEED;
    private final double MAX_ANGULAR_SPEED;

    // Robot swerve modules
    private final TalonFXSwerveModule frontLeft;
    private final TalonFXSwerveModule rearLeft;
    private final TalonFXSwerveModule frontRight;
    private final TalonFXSwerveModule rearRight;
    // The gyro sensor
    private final Pigeon2 gyro;

    private final SwerveDriveKinematics kinematics;
    private final Field2d field = new Field2d();
    // Odometry class for tracking robot pose
    private final SwerveDriveOdometry odometry;

    /**
     * Creates a new DriveSubsystem.
     */
    public Zwerve(SwerveConfig config) {
        this.ROBOT_WIDTH = config.ROBOT_WIDTH;
        this.ROBOT_LENGTH = config.ROBOT_LENGTH;
        this.MAX_SPEED = config.MAX_SPEED;
        this.MAX_ANGULAR_SPEED = config.MAX_ANGULAR_SPEED;
        this.frontLeft = config.frontLeft;
        this.frontRight = config.frontRight;
        this.rearLeft = config.rearLeft;
        this.rearRight = config.rearRight;
        this.gyro = new Pigeon2(config.gyroId);

        kinematics = new SwerveDriveKinematics(new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
                new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
                new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
                new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2));

        this.gyro.getConfigurator().apply(new Pigeon2Configuration());
        this.zeroHeading();

        // By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
        Timer.delay(1.0);
        this.resetEncoders();

        this.odometry = new SwerveDriveOdometry(kinematics, this.getYaw(), this.getModulePositions());
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        this.gyro.reset();
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        frontLeft.resetEncoder();
        rearLeft.resetEncoder();
        frontRight.resetEncoder();
        rearRight.resetEncoder();
    }

    public Rotation2d getYaw() {
        return this.gyro.getRotation2d();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{this.frontLeft.getPosition(), this.frontRight.getPosition(),
                this.rearLeft.getPosition(), this.rearRight.getPosition()};
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        this.odometry.update(this.getYaw(), this.getModulePositions());
        this.field.setRobotPose(this.getPose());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{this.frontLeft.getState(), this.frontRight.getState(), this.rearLeft.getState(),
                this.rearRight.getState()};
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired TalonFXSwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        this.odometry.resetPosition(this.getYaw(), getModulePositions(), pose);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);
        this.setModuleStates(states);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        translation = translation.times(MAX_SPEED);
        rotation = rotation * MAX_ANGULAR_SPEED;
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                        getYaw()) : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        this.setModuleStates(swerveModuleStates);
    }

    public static class SwerveConfig {
        // Distance between centers of right and left wheels on robot
        public double ROBOT_WIDTH = 0;
        // Distance between front and back wheels on robot
        public double ROBOT_LENGTH = 0;
        public double MAX_SPEED = 0;
        public double MAX_ANGULAR_SPEED = 0;

        // Robot swerve modules
        public TalonFXSwerveModule frontLeft = null;
        public TalonFXSwerveModule rearLeft = null;
        public TalonFXSwerveModule frontRight = null;
        public TalonFXSwerveModule rearRight = null;
        // The gyro sensor
        public int gyroId = 0;
    }
}
