package frc.libzodiac.drivetrain;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.libzodiac.api.Gyro;
import frc.libzodiac.api.SwerveDrivetrain;
import frc.libzodiac.hardware.Limelight;
import frc.libzodiac.hardware.group.TalonFXSwerveModule;
import frc.libzodiac.util.GameUtil;
import frc.libzodiac.util.Maths;
import frc.libzodiac.util.Rotation2dSupplier;

import java.util.Collection;
import java.util.HashSet;
import java.util.Optional;

public class TalonFXSwerve implements SwerveDrivetrain {
    public final Config config;
    // Robot swerve modules
    private final TalonFXSwerveModule frontLeft;
    private final TalonFXSwerveModule rearLeft;
    private final TalonFXSwerveModule frontRight;
    private final TalonFXSwerveModule rearRight;
    // The gyro sensor
    private final Gyro gyro;
    private final SwerveDriveKinematics kinematics;
    private final Field2d field = new Field2d();
    private final SwerveDrivePoseEstimator poseEstimator;
    private boolean fieldCentric = true;
    private boolean directAngle = true;
    private boolean slowMode = false;
    private Rotation2d targetHeading = new Rotation2d();

    /**
     * Creates a new DriveSubsystem.
     */
    public TalonFXSwerve(Config config) {
        this.config = config;
        this.frontLeft = config.frontLeft.build(config);
        this.frontRight = config.frontRight.build(config);
        this.rearLeft = config.rearLeft.build(config);
        this.rearRight = config.rearRight.build(config);
        this.gyro = config.gyro;

        this.kinematics = new SwerveDriveKinematics(
                new Translation2d(this.config.robotLength.in(Units.Meters) / 2,
                                  this.config.robotWidth.in(Units.Meters) / 2),
                new Translation2d(this.config.robotLength.in(Units.Meters) / 2,
                                  -this.config.robotWidth.in(Units.Meters) / 2),
                new Translation2d(-this.config.robotLength.in(Units.Meters) / 2,
                                  this.config.robotWidth.in(Units.Meters) / 2),
                new Translation2d(-this.config.robotLength.in(Units.Meters) / 2,
                                  -this.config.robotWidth.in(Units.Meters) / 2));

        this.gyro.reset();

        // By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
        Timer.delay(0.5);
        this.resetEncoders();
        Timer.delay(0.5);

        this.poseEstimator = new SwerveDrivePoseEstimator(this.kinematics, this.getGyroYaw(),
                                                          this.getModulePositions(),
                                                          config.initialPose);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        this.frontLeft.resetEncoder();
        this.rearLeft.resetEncoder();
        this.frontRight.resetEncoder();
        this.rearRight.resetEncoder();
    }

    /**
     * Returns the current yaw reported by the gyro.
     *
     * @return The current yaw reported by the gyro.
     */
    private Rotation2d getGyroYaw() {
        return this.gyro.getRotation2d();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                this.frontLeft.getPosition(), this.frontRight.getPosition(),
                this.rearLeft.getPosition(), this.rearRight.getPosition()
        };
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        this.poseEstimator.update(this.getGyroYaw(), this.getModulePositions());
        Limelight.update();
        this.field.setRobotPose(this.getPose());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    @Override
    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    @Override
    public void setPose(Pose2d pose) {
        this.poseEstimator.resetPosition(this.getGyroYaw(), this.getModulePositions(), pose);
    }

    @Override
    public ChassisSpeeds getRobotCentricSpeeds() {
        return this.kinematics.toChassisSpeeds(this.getModuleStates().orElseThrow());
    }

    @Override
    public void driveRobotCentric(ChassisSpeeds speeds) {
        var states = PathPlanner.generateSwerveSetpoint(speeds);
        if (states == null) {
            SwerveModuleState[] swerveModuleStates = this.kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                                                        this.config.constraints.maxVelocity());
            this.setModuleStates(swerveModuleStates);
        } else {
            this.setModuleStates(states.moduleStates());
        }
    }

    @Override
    public PPHolonomicDriveController getPathFollowingController() {
        return new PPHolonomicDriveController(
                // Translation PID constants
                new PIDConstants(10, 0, 0, 1),
                // Rotation PID constants
                new PIDConstants(this.config.headingPID.getP(), this.config.headingPID.getI(),
                                 this.config.headingPID.getD()));
    }

    @Override
    public Field2d getField() {
        return this.field;
    }

    @Override
    public double getMaxAngularVelocity() {
        return this.config.constraints.maxAngularVelocity().in(Units.RadiansPerSecond);
    }

    @Override
    public Optional<SwerveModuleState[]> getModuleStates() {
        final var state = new SwerveModuleState[]{
                this.frontLeft.getState(), this.frontRight.getState(), this.rearLeft.getState(),
                this.rearRight.getState()
        };
        return Optional.of(state);
    }

    /**
     * Sets the swerve module states.
     *
     * @param desiredStates The desired swerve module states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        this.frontLeft.setDesiredState(desiredStates[0]);
        this.frontRight.setDesiredState(desiredStates[1]);
        this.rearLeft.setDesiredState(desiredStates[2]);
        this.rearRight.setDesiredState(desiredStates[3]);
    }

    @Override
    public void shutdown() {
        this.frontLeft.shutdown();
        this.frontRight.shutdown();
        this.rearLeft.shutdown();
        this.rearRight.shutdown();
    }

    @Override
    public void brake() {
        this.frontLeft.brake();
        this.frontRight.brake();
        this.rearLeft.brake();
        this.rearRight.brake();
    }

    @Override
    public AngularVelocity getAngularVelocity() {
        return this.gyro.getYawAngularVelocity();
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
                                     Matrix<N3, N1> visionMeasurementStdDevs) {
        this.poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds,
                                                visionMeasurementStdDevs);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Drivetrain");
        builder.setActuator(true);
        builder.setSafeState(this::brake);
        builder.addBooleanProperty("Field Centric", this::getFieldCentric, this::setFieldCentric);
        builder.addBooleanProperty("Direct Angle", this::getDirectAngle, this::setDirectAngle);
        builder.addBooleanProperty("Slow Mode", this::getSlowMode, this::setSlowMode);
        SmartDashboard.putData("Swerve Drive", swerveBuilder -> {
            swerveBuilder.setSmartDashboardType("SwerveDrive");

            swerveBuilder.addDoubleProperty("Front Left Angle",
                                            () -> this.frontLeft.getState().angle.getRadians(),
                                            null);
            swerveBuilder.addDoubleProperty("Front Left Velocity",
                                            () -> this.frontLeft.getState().speedMetersPerSecond,
                                            null);

            swerveBuilder.addDoubleProperty("Front Right Angle",
                                            () -> this.frontRight.getState().angle.getRadians(),
                                            null);
            swerveBuilder.addDoubleProperty("Front Right Velocity",
                                            () -> this.frontRight.getState().speedMetersPerSecond,
                                            null);

            swerveBuilder.addDoubleProperty("Back Left Angle",
                                            () -> this.rearLeft.getState().angle.getRadians(),
                                            null);
            swerveBuilder.addDoubleProperty("Back Left Velocity",
                                            () -> this.rearLeft.getState().speedMetersPerSecond,
                                            null);

            swerveBuilder.addDoubleProperty("Back Right Angle",
                                            () -> this.rearRight.getState().angle.getRadians(),
                                            null);
            swerveBuilder.addDoubleProperty("Back Right Velocity",
                                            () -> this.rearRight.getState().speedMetersPerSecond,
                                            null);

            swerveBuilder.addDoubleProperty("Robot Angle", () -> this.getYawRelative().getRadians(),
                                            null);
        });
        SmartDashboard.putData("Reset Heading",
                               Commands.runOnce(this::zeroHeading).ignoringDisable(true));
    }

    @Override
    public void zeroHeading() {
        var pose = this.getPose();
        this.setPose(new Pose2d(pose.getTranslation(), GameUtil.toPose2dYaw(new Rotation2d())));
        this.targetHeading = new Rotation2d();
    }

    @Override
    public boolean getFieldCentric() {
        return this.fieldCentric;
    }

    @Override
    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void toggleFieldCentric() {
        this.fieldCentric = !this.fieldCentric;
    }

    @Override
    public boolean getDirectAngle() {
        return this.directAngle;
    }

    @Override
    public void setDirectAngle(boolean directAngle) {
        if (!this.directAngle && directAngle) {
            this.targetHeading = this.getYawRelative();
        }
        this.directAngle = directAngle;
    }

    @Override
    public void toggleDirectAngle() {
        if (!this.directAngle) {
            this.targetHeading = this.getYawRelative();
        }
        this.directAngle = !this.directAngle;
    }

    @Override
    public boolean getSlowMode() {
        return this.slowMode;
    }

    @Override
    public void setSlowMode(boolean slowMode) {
        this.slowMode = slowMode;
    }

    @Override
    public void toggleSlowMode() {
        this.slowMode = !this.slowMode;
    }

    @Override
    public ChassisSpeeds calculateChassisSpeeds(Translation2d translation, double rotation) {
        final var velocity = Maths.limitTranslation(translation, 1)
                                  .times((this.slowMode ?
                                                  this.config.constraints.maxVelocity().div(3) :
                                                  this.config.constraints.maxVelocity()).in(
                                          Units.MetersPerSecond));
        return new ChassisSpeeds(velocity.getX(), velocity.getY(), rotation *
                                                                   this.config.constraints.maxAngularVelocity()
                                                                                          .in(Units.RadiansPerSecond));
    }

    @Override
    public double calculateRotation(Rotation2dSupplier headingSupplier) {
        this.targetHeading = headingSupplier.asTranslation().getNorm() < 0.5 ? this.targetHeading :
                                     headingSupplier.get();
        return MathUtil.applyDeadband(MathUtil.clamp(this.config.headingPID.calculate(
                this.getYawRelative().minus(this.targetHeading).getRadians(), 0), -1, 1), 0.02);
    }

    @Override
    public Collection<TalonFX> getTalonFXMotors() {
        Collection<TalonFX> motors = new HashSet<>();
        motors.add(this.frontLeft.getAngleMotor().getMotor());
        motors.add(this.frontLeft.getDriveMotor().getMotor());
        motors.add(this.frontRight.getAngleMotor().getMotor());
        motors.add(this.frontRight.getDriveMotor().getMotor());
        motors.add(this.rearLeft.getAngleMotor().getMotor());
        motors.add(this.rearLeft.getDriveMotor().getMotor());
        motors.add(this.rearRight.getAngleMotor().getMotor());
        motors.add(this.rearRight.getDriveMotor().getMotor());
        return motors;
    }

    @Override
    public void setTargetHeading(Rotation2d targetHeading) {
        this.targetHeading = targetHeading;
    }

    public static class Config {
        /**
         * Distance between centers of right and left wheels on robot.
         */
        public Distance robotWidth;
        /**
         * Distance between front and back wheels on robot
         */
        public Distance robotLength;
        public PathConstraints constraints;
        public TalonFXSwerveModule.Config frontLeft;
        public TalonFXSwerveModule.Config rearLeft;
        public TalonFXSwerveModule.Config frontRight;
        public TalonFXSwerveModule.Config rearRight;
        /**
         * The gyroscope.
         */
        public Gyro gyro;
        /**
         * Robot heading
         */
        public PIDController headingPID;
        /**
         * Angle motor's gear ratio, i.e. how many rotations of angle motor produce one revolution
         * of wheel.
         */
        public double angleGearRatio;
        /**
         * Drive motor's gear ratio, i.e. how many rotations of drive motor produce one rotation of
         * wheel.
         */
        public double driveGearRatio;
        /**
         * Radius of wheel.
         */
        public Distance wheelRadius;
        public Pose2d initialPose;
        /**
         * PID arguments shall be set separately for each module, these values serve as a fallback.
         */
        public Slot0Configs driveConfig;
        /**
         * PID arguments shall be set separately for each module, these values serve as a fallback.
         */
        public Slot0Configs angleConfig;

        public Config withInitialPose(Pose2d initialPose) {
            this.initialPose = initialPose;
            return this;
        }

        public Config withRobotLength(Distance robotLength) {
            this.robotLength = robotLength;
            return this;
        }

        public Config withConstraints(PathConstraints constraints) {
            this.constraints = constraints;
            return this;
        }

        public Config withFrontLeft(TalonFXSwerveModule.Config frontLeft) {
            this.frontLeft = frontLeft;
            return this;
        }

        public Config withRearLeft(TalonFXSwerveModule.Config rearLeft) {
            this.rearLeft = rearLeft;
            return this;
        }

        public Config withFrontRight(TalonFXSwerveModule.Config frontRight) {
            this.frontRight = frontRight;
            return this;
        }

        public Config withRearRight(TalonFXSwerveModule.Config rearRight) {
            this.rearRight = rearRight;
            return this;
        }

        public Config withGyro(Gyro gyro) {
            this.gyro = gyro;
            return this;
        }

        public Config withHeadingPID(PIDController headingPID) {
            this.headingPID = headingPID;
            return this;
        }

        public Config withAngleGearRatio(double angleGearRatio) {
            this.angleGearRatio = angleGearRatio;
            return this;
        }

        public Config withDriveGearRatio(double driveGearRatio) {
            this.driveGearRatio = driveGearRatio;
            return this;
        }

        public Config withWheelRadius(Distance wheelRadius) {
            this.wheelRadius = wheelRadius;
            return this;
        }

        public Config withDriveConfig(Slot0Configs driveConfig) {
            this.driveConfig = driveConfig;
            return this;
        }

        public Config withAngleConfig(Slot0Configs angleConfig) {
            this.angleConfig = angleConfig;
            return this;
        }

        public Config withRobotWidth(Distance robotWidth) {
            this.robotWidth = robotWidth;
            return this;
        }

        public TalonFXSwerve build() {
            return new TalonFXSwerve(this);
        }
    }
}
