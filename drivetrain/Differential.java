package frc.libzodiac.drivetrain;

import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.api.Drivetrain;
import frc.libzodiac.api.Gyro;
import frc.libzodiac.hardware.Limelight;
import frc.libzodiac.hardware.TalonSRXMotor;
import frc.libzodiac.util.GameUtil;
import frc.libzodiac.util.Maths;
import frc.libzodiac.util.Rotation2dSupplier;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A differential drive drivetrain.
 */
public final class Differential extends SubsystemBase implements Drivetrain {
    public final Config config;

    /**
     * The left and right motors that control the robot's movement.
     */
    private final TalonSRXMotor leftLeader;
    private final TalonSRXMotor rightLeader;

    /**
     * The gyro.
     */
    private final Gyro gyro;

    /**
     * The controller for the robot's heading.
     */
    private final PIDController headingPID;

    /**
     * The kinematics and pose estimator for the robot.
     */
    private final DifferentialDriveKinematics kinematics;
    private final Field2d field = new Field2d();
    private final DifferentialDrivePoseEstimator poseEstimator;

    /**
     * Whether the robot should drive directly towards a target angle.
     */
    private boolean directAngle = true;

    /**
     * Whether the robot is in slow mode.
     */
    private boolean slowMode = false;

    /**
     * Whether the robot should drive directly towards a target angle.
     */
    private boolean directPower = false;

    /**
     * The target heading of the robot.
     */
    private Rotation2d targetHeading = new Rotation2d();

    /**
     * Construct a differential drive object.
     */
    public Differential(Config config) {
        this.config = config;
        this.leftLeader = new TalonSRXMotor(config.leftLeader);
        this.rightLeader = new TalonSRXMotor(config.rightLeader);
        this.gyro = config.gyro;
        this.headingPID = config.headingPID;
        this.kinematics = new DifferentialDriveKinematics(config.robotWidth);

        final var leftFollower = new TalonSRXMotor(config.leftFollower);
        final var rightFollower = new TalonSRXMotor(config.rightFollower);

        /*
          Configure the motors and gyro.
         */
        this.leftLeader.factoryDefault();
        leftFollower.factoryDefault();
        this.rightLeader.factoryDefault();
        rightFollower.factoryDefault();

        this.leftLeader.setPID(config.pidController);
        this.rightLeader.setPID(config.pidController);

        this.leftLeader.setPhase(config.leftEncoderPhase);
        this.rightLeader.setPhase(config.rightEncoderPhase);

        this.leftLeader.resetPosition();
        this.rightLeader.resetPosition();

        // Motors should drive forward when given positive voltage
        this.leftLeader.setInverted(config.leftLeaderInverted);
        this.rightLeader.setInverted(config.rightLeaderInverted);
        leftFollower.follow(this.leftLeader, config.leftFollowerInverted != config.leftLeaderInverted);
        rightFollower.follow(this.rightLeader, config.rightFollowerInverted != config.rightLeaderInverted);

        this.leftLeader.setBrakeWhenNeutral(true);
        this.rightLeader.setBrakeWhenNeutral(true);

        this.gyro.reset();

        var wheelPositions = this.getWheelPositions();
        this.poseEstimator = new DifferentialDrivePoseEstimator(this.kinematics, this.getGyroYaw(),
                wheelPositions.leftMeters, wheelPositions.rightMeters, config.initialPose);
    }

    /**
     * Returns the current wheel position of the robot.
     *
     * @return The current wheel position.
     */
    private DifferentialDriveWheelPositions getWheelPositions() {
        return new DifferentialDriveWheelPositions(
                this.config.wheelRadius.times(this.leftLeader.getPosition().in(Units.Radians)),
                this.config.wheelRadius.times(this.rightLeader.getPosition().in(Units.Radians)));
    }

    /**
     * Returns the current yaw reported by the gyro.
     *
     * @return The current yaw reported by the gyro.
     */
    private Rotation2d getGyroYaw() {
        return this.gyro.getRotation2d();
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed Linear velocity in m/s.
     * @param rot    Angular velocity in rad/s.
     */
    public void drive(double xSpeed, double rot) {
        this.drive(new ChassisSpeeds(xSpeed, 0, rot));
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.setSpeeds(this.kinematics.toWheelSpeeds(chassisSpeeds));
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        // DEBUG
        SmartDashboard.putNumber("Left Desired Speed",
                speeds.leftMetersPerSecond / this.config.wheelRadius.in(Units.Meter));
        SmartDashboard.putNumber("Right Desired Speed",
                speeds.rightMetersPerSecond / this.config.wheelRadius.in(Units.Meter));

        this.leftLeader.setVelocity(
                Units.RadiansPerSecond.of(speeds.leftMetersPerSecond / this.config.wheelRadius.in(Units.Meter)));
        this.rightLeader.setVelocity(
                Units.RadiansPerSecond.of(speeds.rightMetersPerSecond / this.config.wheelRadius.in(Units.Meter)));
    }

    /**
     * Returns a command that drives the robot with the given linear velocity and angular velocity.
     *
     * @param directAngle      Direct Angle Mode Input.
     * @param angularVelocity  Angular Velocity Mode Input.
     * @param driveDirectAngle Whether the robot should drive directly towards a target angle.
     *
     * @return The command that drives the robot with the given linear velocity and angular velocity.
     */
    public Command getDriveCommand(Supplier<ChassisSpeeds> directAngle, Supplier<ChassisSpeeds> angularVelocity,
            BooleanSupplier driveDirectAngle, BooleanSupplier directPower) {
        return run(() -> {
            if (directPower.getAsBoolean()) {
                this.driveDirectPower(driveDirectAngle.getAsBoolean() ? directAngle.get() : angularVelocity.get());
            } else {
                this.drive(driveDirectAngle.getAsBoolean() ? directAngle.get() : angularVelocity.get());
            }
        });
    }

    public void driveDirectPower(ChassisSpeeds chassisSpeeds) {
        var wheelSpeeds = this.kinematics.toWheelSpeeds(chassisSpeeds);
        final var MAX_SPEED = this.config.maxSpeed.in(Units.MetersPerSecond);
        this.leftLeader.setPower(wheelSpeeds.leftMetersPerSecond / MAX_SPEED);
        this.rightLeader.setPower(wheelSpeeds.rightMetersPerSecond / MAX_SPEED);
    }

    public void driveDirectPower(double xSpeed, double rot) {
        this.driveDirectPower(new ChassisSpeeds(xSpeed, 0, rot));
    }

    @Override
    public void periodic() {
        /*
          Update the pose using wheel position and Limelight.
         */
        this.poseEstimator.update(this.getGyroYaw(), this.getWheelPositions());
        Limelight.update();
        this.field.setRobotPose(this.getPose());
    }

    /**
     * Initializes the sendable for the drivetrain.
     *
     * @param builder The sendable builder.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Differential Drivetrain");
        builder.setActuator(true);
        builder.setSafeState(this::brake);
        builder.addBooleanProperty("Direct Angle", this::getDirectAngle, this::setDirectAngle);
        builder.addBooleanProperty("Slow Mode", this::getSlowMode, this::setSlowMode);
        builder.addBooleanProperty("Direct Power", this::getDirectPower, this::setDirectPower);
        builder.addDoubleProperty("Heading", () -> -this.getYawRelative().getDegrees(), null);
        SmartDashboard.putData("Differential Drive", differentialBuilder -> {
            differentialBuilder.setSmartDashboardType("DifferentialDrive");
            differentialBuilder.addDoubleProperty("Left Motor Speed", () -> this.getWheelSpeeds().leftMetersPerSecond,
                    null);
            differentialBuilder.addDoubleProperty("Right Motor Speed", () -> this.getWheelSpeeds().rightMetersPerSecond,
                    null);
        });
        SmartDashboard.putData("Reset Heading", Commands.runOnce(this::zeroHeading).ignoringDisable(true));
    }

    /**
     * Returns whether the robot should drive directly towards a target angle.
     *
     * @return Whether the robot should drive directly towards a target angle.
     */
    public boolean getDirectAngle() {
        return this.directAngle;
    }

    /**
     * Sets whether the robot should drive directly towards a target angle.
     *
     * @param directAngle Whether the robot should drive directly towards a target angle.
     */
    public void setDirectAngle(boolean directAngle) {
        this.directAngle = directAngle;
    }

    /**
     * Returns whether the robot is in slow mode.
     *
     * @return Whether the robot is in slow mode.
     */
    public boolean getSlowMode() {
        return this.slowMode;
    }

    /**
     * Set whether the robot is in slow mode.
     */
    public void setSlowMode(boolean slowMode) {
        this.slowMode = slowMode;
    }

    /**
     * Returns whether the robot is in direct power mode.
     *
     * @return Whether the robot is in direct power mode.
     */
    public boolean getDirectPower() {
        return this.directPower;
    }

    /**
     * Set whether the robot is in direct power mode.
     */
    public void setDirectPower(boolean directPower) {
        this.directPower = directPower;
    }

    /**
     * Returns the current yaw of the robot. The yaw of 0 is the robot facing directly away from your alliance station
     * wall.
     *
     * @return The current yaw.
     */
    public Rotation2d getYawRelative() {
        return GameUtil.toAllianceRelativeYaw(this.getYaw());
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    private DifferentialDriveWheelSpeeds getWheelSpeeds() {

        return new DifferentialDriveWheelSpeeds(
                this.leftLeader.getVelocity().asFrequency().times(this.config.wheelRadius.times(2 * Math.PI)),
                this.rightLeader.getVelocity().asFrequency().times(this.config.wheelRadius.times(2 * Math.PI)));
    }

    /**
     * Zero the heading of the robot.
     */
    public void zeroHeading() {
        var pose = this.getPose();
        this.setPose(new Pose2d(pose.getTranslation(), GameUtil.toPose2dYaw(new Rotation2d())));
    }

    /**
     * Returns the current yaw of the robot. The yaw of 0 is the robot facing the red alliance wall.
     *
     * @return The current yaw.
     */
    public Rotation2d getYaw() {
        return this.getPose().getRotation();
    }

    /**
     * Sets the motor brake mode.
     *
     * @param brake Whether to brake the motors.
     */
    public void setMotorBrake(boolean brake) {
        if (brake) {
            this.leftLeader.brake();
            this.rightLeader.brake();
        } else {
            this.leftLeader.shutdown();
            this.rightLeader.shutdown();
        }
    }

    /**
     * Toggles direct angle mode.
     */
    public void toggleDirectAngle() {
        this.directAngle = !this.directAngle;
    }

    /**
     * Toggles slow mode.
     */
    public void toggleSlowMode() {
        this.slowMode = !this.slowMode;
    }

    /**
     * Calculates the desired rotation of the robot in direct angle mode.
     *
     * @param headingSupplier The heading supplier.
     *
     * @return The rotation.
     */
    private double calculateRotation(Rotation2dSupplier headingSupplier) {
        this.targetHeading =
                headingSupplier.asTranslation().getNorm() < 0.5 ? this.targetHeading : headingSupplier.get();
        return MathUtil.applyDeadband(MathUtil.clamp(
                        this.headingPID.calculate(this.getYawRelative().minus(this.targetHeading).getRadians(), 0), -1, 1),
                0.05);
    }

    /**
     * Returns speeds for the robot based on the given velocity and rotation.
     *
     * @param velocity velocity in [-1, 1].
     * @param rotation Angular velocity in [-1, 1].
     */
    public ChassisSpeeds getChassisSpeeds(double velocity, double rotation) {
        return new ChassisSpeeds((this.slowMode ? this.config.maxSpeed.div(2) : this.config.maxSpeed).times(velocity),
                Units.MetersPerSecond.of(0),
                (this.slowMode ? this.config.maxAngularVelocity.div(1.5) : this.config.maxAngularVelocity).times(
                        rotation));
    }

    @Override
    public DifferentialDrivePoseEstimator getPoseEstimator() {
        return this.poseEstimator;
    }

    @Override
    public Gyro getGyro() {
        return this.gyro;
    }

    @Override
    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    @Override
    public void setPose(Pose2d pose) {
        this.poseEstimator.resetPosition(this.getGyroYaw(), this.getWheelPositions(), pose);
    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.kinematics.toChassisSpeeds(this.getWheelSpeeds());
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        this.drive(chassisSpeeds);
    }

    @Override
    public PPLTVController getPathFollowingController() {
        return new PPLTVController(0.02, this.config.maxSpeed.in(Units.MetersPerSecond));
    }

    @Override
    public Field2d getField() {
        return this.field;
    }

    @Override
    public double getMaxAngularVelocity() {
        return this.config.maxAngularVelocity.in(Units.RadiansPerSecond);
    }

    @Override
    public Optional<SwerveModuleState[]> getModuleStates() {
        return Optional.empty();
    }

    @Override
    public void shutdown() {
        this.leftLeader.shutdown();
        this.rightLeader.shutdown();
    }

    /**
     * Brake the robot.
     */
    @Override
    public void brake() {
        this.leftLeader.brake();
        this.rightLeader.brake();
    }

    /**
     * Config for the differential drive.
     */
    public static class Config {

        /**
         * The width of the robot in meters.
         */
        public Distance robotWidth;
        /**
         * The maximum speed of the robot in m/s.
         */
        public LinearVelocity maxSpeed;
        /**
         * The maximum turning speed of the robot in rad/s.
         */
        public AngularVelocity maxAngularVelocity;
        /**
         * The radius of the wheels in meters.
         */
        public Distance wheelRadius;
        /**
         * The IDs of the motors.
         */
        public int leftLeader;
        public int leftFollower;
        public int rightLeader;
        public int rightFollower;
        /**
         * Whether the motors are inverted.
         */
        public boolean leftLeaderInverted = false;
        public boolean leftFollowerInverted = false;
        public boolean rightLeaderInverted = false;
        public boolean rightFollowerInverted = false;
        /**
         * Whether the encoders are phase inverted.
         */
        public boolean leftEncoderPhase;
        public boolean rightEncoderPhase;
        /**
         * The ID of the gyro.
         */
        public Gyro gyro;
        /**
         * The PID controller for the motors.
         */
        public PIDController pidController;
        /**
         * The PID controller for the heading.
         */
        public PIDController headingPID;
        public Pose2d initialPose;

        public Differential build() {
            return new Differential(this);
        }

        public Config withInitialPose(Pose2d initialPose) {
            this.initialPose = initialPose;
            return this;
        }

        public Config withRobotWidth(Distance robotWidth) {
            this.robotWidth = robotWidth;
            return this;
        }

        public Config withMaxSpeed(LinearVelocity maxSpeed) {
            this.maxSpeed = maxSpeed;
            return this;
        }

        public Config withMaxAngularVelocity(AngularVelocity maxAngularVelocity) {
            this.maxAngularVelocity = maxAngularVelocity;
            return this;
        }

        public Config withWheelRadius(Distance wheelRadius) {
            this.wheelRadius = wheelRadius;
            return this;
        }

        public Config withLeftLeader(int leftLeader) {
            this.leftLeader = leftLeader;
            return this;
        }

        public Config withLeftFollower(int leftFollower) {
            this.leftFollower = leftFollower;
            return this;
        }

        public Config withRightLeader(int rightLeader) {
            this.rightLeader = rightLeader;
            return this;
        }

        public Config withRightFollower(int rightFollower) {
            this.rightFollower = rightFollower;
            return this;
        }

        public Config withLeftLeaderInverted(boolean leftLeaderInverted) {
            this.leftLeaderInverted = leftLeaderInverted;
            return this;
        }

        public Config withRightLeaderInverted(boolean rightLeaderInverted) {
            this.rightLeaderInverted = rightLeaderInverted;
            return this;
        }

        public Config withLeftFollowerInverted(boolean leftFollowerInverted) {
            this.leftFollowerInverted = leftFollowerInverted;
            return this;
        }

        public Config withRightFollowerInverted(boolean rightFollowerInverted) {
            this.rightFollowerInverted = rightFollowerInverted;
            return this;
        }

        public Config withLeftEncoderPhase(boolean leftEncoderPhase) {
            this.leftEncoderPhase = leftEncoderPhase;
            return this;
        }

        public Config withRightEncoderPhase(boolean rightEncoderPhase) {
            this.rightEncoderPhase = rightEncoderPhase;
            return this;
        }

        public Config withGyro(Gyro gyro) {
            this.gyro = gyro;
            return this;
        }

        public Config withPidController(PIDController pidController) {
            this.pidController = pidController;
            return this;
        }

        public Config withHeadingPID(PIDController headingPID) {
            this.headingPID = headingPID;
            return this;
        }
    }

    /**
     * An input stream for the drivetrain.
     */
    public static class InputStream implements Supplier<ChassisSpeeds> {
        /**
         * The drivetrain.
         */
        private final Differential drivetrain;
        /**
         * The velocity supplier.
         */
        private final DoubleSupplier velocity;
        /**
         * The deadband.
         */
        private double deadband = 0;
        /**
         * The rotation type.
         */
        private RotationType rotationType = RotationType.NONE;
        /**
         * The heading supplier.
         */
        private Rotation2dSupplier heading;
        /**
         * The rotation supplier.
         */
        private DoubleSupplier rotation;

        /**
         * Constructs an input stream for the drivetrain.
         *
         * @param drivetrain The drivetrain.
         * @param velocity   The velocity supplier.
         */
        public InputStream(Differential drivetrain, DoubleSupplier velocity) {
            this.drivetrain = drivetrain;
            this.velocity = velocity;
        }

        /**
         * Sets the rotation supplier.
         *
         * @param rotation The rotation supplier.
         *
         * @return The input stream.
         */
        public InputStream rotation(DoubleSupplier rotation) {
            this.rotation = rotation;
            this.rotationType = InputStream.RotationType.ROTATION;
            return this;
        }

        /**
         * Sets the deadband.
         *
         * @param deadband The deadband.
         *
         * @return The input stream.
         */
        public InputStream deadband(double deadband) {
            this.deadband = deadband;
            return this;
        }

        /**
         * Sets the heading supplier.
         *
         * @param heading The heading supplier.
         *
         * @return The input stream.
         */
        public InputStream heading(Rotation2dSupplier heading) {
            this.heading = heading;
            this.rotationType = InputStream.RotationType.HEADING;
            return this;
        }

        /**
         * Returns the speeds for the drivetrain.
         *
         * @return The speeds.
         */
        @Override
        public ChassisSpeeds get() {
            var velocity = Maths.square(MathUtil.applyDeadband(this.velocity.getAsDouble(), this.deadband));
            var rotation = switch (this.rotationType) {
                case HEADING -> this.drivetrain.calculateRotation(this.heading);
                case ROTATION -> Maths.square(MathUtil.applyDeadband(this.rotation.getAsDouble(), this.deadband));
                case NONE -> 0;
            };
            return this.drivetrain.getChassisSpeeds(velocity, rotation);
        }

        /**
         * The rotation type.
         */
        enum RotationType {
            HEADING,
            ROTATION,
            NONE
        }
    }
}
