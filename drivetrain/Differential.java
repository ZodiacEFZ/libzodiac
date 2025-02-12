package frc.libzodiac.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.api.DrivetrainBase;
import frc.libzodiac.hardware.Limelight;
import frc.libzodiac.hardware.TalonSRXMotor;
import frc.libzodiac.util.Maths;
import frc.libzodiac.util.Rotation2dSupplier;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Differential extends SubsystemBase implements DrivetrainBase {
    public final double MAX_SPEED;
    public final double MAX_ANGULAR_VELOCITY;
    private final double GEAR_RATIO;
    private final double WHEEL_RADIUS; // meters

    private final TalonSRXMotor leftLeader;
    private final TalonSRXMotor leftFollower;
    private final TalonSRXMotor rightLeader;
    private final TalonSRXMotor rightFollower;

    private final Pigeon2 gyro;

    private final PIDController headingController;
    private final DifferentialDriveKinematics kinematics;
    private final Field2d field = new Field2d();
    private final DifferentialDrivePoseEstimator poseEstimator;
    private boolean directAngle = true;
    private Rotation2d targetHeading = new Rotation2d();

    /**
     * Constructs a differential drive object. Sets the Encoder distance per pulse and resets the
     * gyro.
     */
    public Differential(Config config, Pose2d initialPose) {
        this.MAX_SPEED = config.MAX_SPEED;
        this.MAX_ANGULAR_VELOCITY = config.MAX_ANGULAR_VELOCITY;
        this.GEAR_RATIO = config.GEAR_RATIO;
        this.WHEEL_RADIUS = config.WHEEL_RADIUS;
        this.leftLeader = new TalonSRXMotor(config.leftLeader);
        this.leftFollower = new TalonSRXMotor(config.leftFollower);
        this.rightLeader = new TalonSRXMotor(config.rightLeader);
        this.rightFollower = new TalonSRXMotor(config.rightFollower);
        this.gyro = new Pigeon2(config.gyro);
        this.headingController = config.headingController;
        this.kinematics = new DifferentialDriveKinematics(config.ROBOT_WIDTH);

        this.leftLeader.factoryDefault();
        this.leftFollower.factoryDefault();
        this.rightLeader.factoryDefault();
        this.rightFollower.factoryDefault();

        this.leftLeader.setPID(config.pidController);
        this.rightLeader.setPID(config.pidController);

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        this.leftLeader.resetPosition();
        this.rightLeader.resetPosition();
        this.leftLeader.setInverted(config.leftLeaderInverted);
        this.rightLeader.setInverted(config.rightLeaderInverted);
        this.leftFollower.follow(this.leftLeader, config.leftFollowerInverted != config.leftLeaderInverted);
        this.rightFollower.follow(this.rightLeader, config.rightFollowerInverted != config.rightLeaderInverted);

        this.leftLeader.setPhase(config.leftEncoderPhase);
        this.rightLeader.setPhase(config.rightEncoderPhase);

        this.gyro.reset();

        var wheelPositions = this.getWheelPositions();
        this.poseEstimator = new DifferentialDrivePoseEstimator(this.kinematics, this.getYaw(),
                wheelPositions.leftMeters, wheelPositions.rightMeters, initialPose);
    }

    private DifferentialDriveWheelPositions getWheelPositions() {
        return new DifferentialDriveWheelPositions(this.leftLeader.getPosition() * this.WHEEL_RADIUS,
                this.rightLeader.getPosition() * this.WHEEL_RADIUS);
    }

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
        SmartDashboard.putNumber("Left Speed", speeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Right Speed",
                speeds.leftMetersPerSecond * this.GEAR_RATIO / this.WHEEL_RADIUS * 4096 / 2 / Math.PI / 10);
        SmartDashboard.putNumber("Left Speed Encoder", this.leftLeader.getVelocity());
        SmartDashboard.putNumber("Right Speed Encoder", this.rightLeader.getVelocity());
        this.leftLeader.velocity(speeds.leftMetersPerSecond * this.GEAR_RATIO / this.WHEEL_RADIUS);
        this.rightLeader.velocity(speeds.rightMetersPerSecond * this.GEAR_RATIO / this.WHEEL_RADIUS);
    }

    public Command getDriveCommand(Supplier<ChassisSpeeds> directAngle, Supplier<ChassisSpeeds> angularVelocity,
                                   BooleanSupplier driveDirectAngle) {
        return run(() -> this.drive(driveDirectAngle.getAsBoolean() ? directAngle.get() : angularVelocity.get()));
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        this.poseEstimator.update(this.getYaw(), this.getWheelPositions());
        Limelight.update();
        this.field.setRobotPose(this.getPose());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Differential Drivetrain");
        builder.setActuator(true);
        builder.addBooleanProperty("Direct Angle", this::getDirectAngle, this::setDirectAngle);
        builder.addDoubleProperty("Heading", () -> -this.getYaw().getDegrees(), null);
        builder.addStringProperty("Pose", () -> this.getPose().getTranslation().toString(), null);

        SmartDashboard.putData("Reset Heading", Commands.runOnce(this::zeroHeading).ignoringDisable(true));
    }

    public boolean getDirectAngle() {
        return this.directAngle;
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        this.gyro.reset();
    }

    public void setDirectAngle(boolean directAngle) {
        this.directAngle = directAngle;
    }

    public void setMotorBrake(boolean brake) {
        if (brake) {
            this.leftLeader.brake();
            this.rightLeader.brake();
        } else {
            this.leftLeader.shutdown();
            this.rightLeader.shutdown();
        }
    }

    public void toggleDirectAngle() {
        this.directAngle = !this.directAngle;
    }

    private double calculateRotation(Rotation2dSupplier headingSupplier) {
        this.targetHeading = headingSupplier.asTranslation()
                .getNorm() < 0.5 ? this.targetHeading : headingSupplier.get();
        return MathUtil.clamp(this.headingController.calculate(this.getYaw().minus(this.targetHeading).getRadians(), 0),
                -1, 1);
    }

    public ChassisSpeeds getChassisSpeeds(double velocity, double rotation) {
        return new ChassisSpeeds(velocity * this.MAX_SPEED, 0, rotation * this.MAX_ANGULAR_VELOCITY);
    }

    @Override
    public DifferentialDrivePoseEstimator getPoseEstimator() {
        return this.poseEstimator;
    }

    @Override
    public Pigeon2 getGyro() {
        return this.gyro;
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
    public Subsystem getSubsystemBase() {
        return this;
    }

    @Override
    public PPLTVController getPathFollowingController() {
        return new PPLTVController(0.02);
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
    public SwerveModuleState[] getModuleStates() {
        return null;
    }

    @Override
    public boolean isSwerve() {
        return false;
    }

    private DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(this.leftLeader.getVelocity() * this.WHEEL_RADIUS / this.GEAR_RATIO,
                this.rightLeader.getVelocity() * this.WHEEL_RADIUS / this.GEAR_RATIO);
    }

    public static class Config {
        public double ROBOT_WIDTH; // meters
        public double MAX_SPEED; // meters per second
        public double MAX_ANGULAR_VELOCITY; // radians per second
        public double GEAR_RATIO;
        public double WHEEL_RADIUS; // meters
        public int leftLeader;
        public int leftFollower;
        public int rightLeader;
        public int rightFollower;
        public boolean leftLeaderInverted;
        public boolean leftFollowerInverted;
        public boolean rightLeaderInverted;
        public boolean rightFollowerInverted;
        public boolean leftEncoderPhase;
        public boolean rightEncoderPhase;
        public int gyro;
        public PIDController pidController;
        public PIDController headingController;
    }

    public static class InputStream implements Supplier<ChassisSpeeds> {
        private final Differential drivetrain;
        private final DoubleSupplier velocity;
        private double deadband = 0;
        private RotationType rotationType = RotationType.NONE;
        private Rotation2dSupplier heading;
        private DoubleSupplier rotation;

        public InputStream(Differential drivetrain, DoubleSupplier velocity) {
            this.drivetrain = drivetrain;
            this.velocity = velocity;
        }

        public InputStream rotation(DoubleSupplier rotation) {
            this.rotation = rotation;
            this.rotationType = InputStream.RotationType.ROTATION;
            return this;
        }

        public InputStream deadband(double deadband) {
            this.deadband = deadband;
            return this;
        }

        public InputStream heading(Rotation2dSupplier heading) {
            this.heading = heading;
            this.rotationType = InputStream.RotationType.HEADING;
            return this;
        }

        @Override
        public ChassisSpeeds get() {
            var velocity = Maths.cube(MathUtil.applyDeadband(this.velocity.getAsDouble(), this.deadband));
            var rotation = switch (this.rotationType) {
                case HEADING -> this.drivetrain.calculateRotation(this.heading);
                case ROTATION -> Maths.cube(MathUtil.applyDeadband(this.rotation.getAsDouble(), this.deadband));
                case NONE -> 0;
            };
            return this.drivetrain.getChassisSpeeds(velocity, rotation);
        }

        enum RotationType {
            HEADING, ROTATION, NONE
        }
    }
}
