package frc.libzodiac.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.api.Drivetrain;
import frc.libzodiac.hardware.Limelight;
import frc.libzodiac.hardware.TalonSRXMotor;
import frc.libzodiac.util.Maths;
import frc.libzodiac.util.Rotation2dSupplier;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A differential drive drivetrain.
 */
public class Differential extends SubsystemBase implements Drivetrain {
    /**
     * The maximum speed of the robot in m/s.
     */
    public final double MAX_SPEED;
    /**
     * The maximum turning speed of the robot in rad/s.
     */
    public final double MAX_ANGULAR_VELOCITY;
    /**
     * The radius of the wheels in meters.
     */
    private final double WHEEL_RADIUS; // meters


    /**
     * The left and right motors that control the robot's movement.
     */
    private final TalonSRXMotor leftLeader;
    private final TalonSRXMotor rightLeader;

    /**
     * The gyro.
     */
    private final Pigeon2 gyro;

    /**
     * The controller for the robot's heading.
     */
    private final PIDController headingController;

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
     * The target heading of the robot.
     */
    private Rotation2d targetHeading = new Rotation2d();

    /**
     * Construct a differential drive object.
     */
    public Differential(Config config, Pose2d initialPose) {
        /*
          Initialize the constants, motors, gyro, and controllers.
         */
        this.MAX_SPEED = config.MAX_SPEED;
        this.MAX_ANGULAR_VELOCITY = config.MAX_ANGULAR_VELOCITY;
        this.WHEEL_RADIUS = config.WHEEL_RADIUS;
        this.leftLeader = new TalonSRXMotor(config.leftLeader);
        TalonSRXMotor leftFollower = new TalonSRXMotor(config.leftFollower);
        this.rightLeader = new TalonSRXMotor(config.rightLeader);
        TalonSRXMotor rightFollower = new TalonSRXMotor(config.rightFollower);
        this.gyro = new Pigeon2(config.gyro);
        this.headingController = config.headingController;
        this.kinematics = new DifferentialDriveKinematics(config.ROBOT_WIDTH);

        /*
          Configure the motors and gyro.
         */
        this.leftLeader.factoryDefault();
        leftFollower.factoryDefault();
        this.rightLeader.factoryDefault();
        rightFollower.factoryDefault();

        this.leftLeader.setPID(config.pidController);
        this.rightLeader.setPID(config.pidController);

        this.leftLeader.resetPosition();
        this.rightLeader.resetPosition();

        // Motors should drive forward when given positive voltage
        this.leftLeader.setInverted(config.leftLeaderInverted);
        this.rightLeader.setInverted(config.rightLeaderInverted);
        leftFollower.follow(this.leftLeader, config.leftFollowerInverted != config.leftLeaderInverted);
        rightFollower.follow(this.rightLeader, config.rightFollowerInverted != config.rightLeaderInverted);

        this.leftLeader.setBrakeWhenNeutral(true);
        this.leftLeader.setBrakeWhenNeutral(true);

        this.leftLeader.setPhase(config.leftEncoderPhase);
        this.rightLeader.setPhase(config.rightEncoderPhase);

        this.gyro.reset();

        var wheelPositions = this.getWheelPositions();
        this.poseEstimator = new DifferentialDrivePoseEstimator(this.kinematics, this.getYaw(),
                wheelPositions.leftMeters, wheelPositions.rightMeters, initialPose);
    }

    /**
     * Returns the current wheel position of the robot.
     *
     * @return The current wheel position.
     */
    private DifferentialDriveWheelPositions getWheelPositions() {
        return new DifferentialDriveWheelPositions(this.leftLeader.getPosition().in(Units.Radians) * this.WHEEL_RADIUS,
                this.rightLeader.getPosition().in(Units.Radians) * this.WHEEL_RADIUS);
    }

    /**
     * Returns the current yaw of the robot.
     *
     * @return The current yaw.
     */
    public Rotation2d getYaw() {
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
        this.leftLeader.velocity(Units.RadiansPerSecond.of(speeds.leftMetersPerSecond / this.WHEEL_RADIUS));
        this.rightLeader.velocity(Units.RadiansPerSecond.of(speeds.rightMetersPerSecond / this.WHEEL_RADIUS));
    }

    /**
     * Returns a command that drives the robot with the given linear velocity and angular velocity.
     *
     * @param directAngle      Direct Angle Mode Input.
     * @param angularVelocity  Angular Velocity Mode Input.
     * @param driveDirectAngle Whether the robot should drive directly towards a target angle.
     * @return The command.
     */
    public Command getDriveCommand(Supplier<ChassisSpeeds> directAngle, Supplier<ChassisSpeeds> angularVelocity, BooleanSupplier driveDirectAngle) {
        return run(() -> this.drive(driveDirectAngle.getAsBoolean() ? directAngle.get() : angularVelocity.get()));
    }

    @Override
    public void periodic() {
        /*
          Update the pose using wheel position and Limelight.
         */
        this.poseEstimator.update(this.getYaw(), this.getWheelPositions());
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
        builder.addBooleanProperty("Slow Mode", () -> this.slowMode, this::setSlowMode);
        builder.addDoubleProperty("Heading", () -> -this.getYaw().getDegrees(), null);
        SmartDashboard.putData("Differential Drive", differentialBuilder -> {
            differentialBuilder.setSmartDashboardType("DifferentialDrive");
            differentialBuilder.addDoubleProperty("Left Motor Speed", () -> this.getWheelSpeeds().leftMetersPerSecond, null);
            differentialBuilder.addDoubleProperty("Right Motor Speed", () -> this.getWheelSpeeds().rightMetersPerSecond, null);
        });
        SmartDashboard.putData("Reset Heading", Commands.runOnce(this::zeroHeading).ignoringDisable(true));
    }

    /**
     * Set whether the robot is in slow mode.
     */
    public void setSlowMode(boolean slowMode) {
        this.slowMode = slowMode;
    }

    /**
     * Brake the robot.
     */
    @Override
    public void brake() {
        this.leftLeader.brake();
        this.rightLeader.brake();
    }

    @Override
    public void shutdown() {
        this.leftLeader.shutdown();
        this.rightLeader.shutdown();
    }

    /**
     * Zero the heading of the robot.
     */
    public void zeroHeading() {
        this.gyro.reset();
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
     * @return The rotation.
     */
    private double calculateRotation(Rotation2dSupplier headingSupplier) {
        this.targetHeading = headingSupplier.asTranslation()
                .getNorm() < 0.5 ? this.targetHeading : headingSupplier.get();
        return MathUtil.applyDeadband(MathUtil.clamp(this.headingController.calculate(this.getYaw().minus(this.targetHeading).getRadians(), 0),
                -1, 1), 0.05);
    }

    /**
     * Returns speeds for the robot based on the given velocity and rotation.
     *
     * @param velocity velocity in [-1, 1].
     * @param rotation Angular velocity in [-1, 1].
     */
    public ChassisSpeeds getChassisSpeeds(double velocity, double rotation) {
        return new ChassisSpeeds(velocity * (this.slowMode ? this.MAX_SPEED / 5 : this.MAX_SPEED), 0, rotation * (this.slowMode ? this.MAX_ANGULAR_VELOCITY / 3 : this.MAX_ANGULAR_VELOCITY));
    }

    @Override
    public DifferentialDrivePoseEstimator getPoseEstimator() {
        return this.poseEstimator;
    }

    @Override
    public Pigeon2 getGyro() {
        return this.gyro;
    }

    @Override
    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    @Override
    public void setPose(Pose2d pose) {
        this.poseEstimator.resetPosition(this.getYaw(), this.getWheelPositions(), pose);
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
        return new PPLTVController(0.02, this.MAX_SPEED);
    }

    @Override
    public Field2d getField() {
        return this.field;
    }

    @Override
    public double getMaxAngularVelocity() {
        return this.MAX_ANGULAR_VELOCITY;
    }

    @Override
    public Optional<SwerveModuleState[]> getModuleStates() {
        return Optional.empty();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    private DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                this.leftLeader.getVelocity().in(Units.RadiansPerSecond) * this.WHEEL_RADIUS,
                this.rightLeader.getVelocity().in(Units.RadiansPerSecond) * this.WHEEL_RADIUS);
    }

    /**
     * Config for the differential drive.
     */
    public static class Config {
        /**
         * The width of the robot in meters.
         */
        public double ROBOT_WIDTH;
        /**
         * The maximum speed of the robot in m/s.
         */
        public double MAX_SPEED;
        /**
         * The maximum turning speed of the robot in rad/s.
         */
        public double MAX_ANGULAR_VELOCITY;
        /**
         * The radius of the wheels in meters.
         */
        public double WHEEL_RADIUS;
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
        public boolean leftLeaderInverted;
        public boolean leftFollowerInverted;
        public boolean rightLeaderInverted;
        public boolean rightFollowerInverted;
        /**
         * Whether the encoders are phase inverted.
         */
        public boolean leftEncoderPhase;
        public boolean rightEncoderPhase;
        /**
         * The ID of the gyro.
         */
        public int gyro;
        /**
         * The PID controller for the motors.
         */
        public PIDController pidController;
        /**
         * The PID controller for the heading.
         */
        public PIDController headingController;
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
            HEADING, ROTATION, NONE
        }
    }
}
