package frc.libzodiac.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.MagEncoder;
import frc.libzodiac.hardware.TalonSRXMotor;
import frc.libzodiac.util.Maths;
import frc.libzodiac.util.Rotation2dSupplier;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ZDifferential extends SubsystemBase implements Sendable {
    public final double MAX_SPEED;
    public final double MAX_ANGULAR_SPEED;
    private final double GEAR_RATIO;
    private final double WHEEL_RADIUS; // meters

    private final TalonSRXMotor leftLeader;
    private final TalonSRXMotor leftFollower;
    private final TalonSRXMotor rightLeader;
    private final TalonSRXMotor rightFollower;
    private final MagEncoder leftEncoder;
    private final MagEncoder rightEncoder;
    private final Pigeon2 gyro;

    private final PIDController headingController;
    private final DifferentialDriveKinematics kinematics;
    private final Field2d field = new Field2d();
    private final DifferentialDriveOdometry odometry;
    private boolean directAngle = true;
    private Rotation2d targetHeading = new Rotation2d();

    /**
     * Constructs a differential drive object. Sets the Encoder distance per pulse and resets the
     * gyro.
     */
    public ZDifferential(Config config) {
        this.MAX_SPEED = config.MAX_SPEED;
        this.MAX_ANGULAR_SPEED = config.MAX_ANGULAR_SPEED;
        this.GEAR_RATIO = config.GEAR_RATIO;
        this.WHEEL_RADIUS = config.WHEEL_RADIUS;
        this.leftLeader = new TalonSRXMotor(config.leftLeader);
        this.leftFollower = new TalonSRXMotor(config.leftFollower);
        this.rightLeader = new TalonSRXMotor(config.rightLeader);
        this.rightFollower = new TalonSRXMotor(config.rightFollower);
        this.leftEncoder = new MagEncoder(config.leftEncoder);
        this.rightEncoder = new MagEncoder(config.rightEncoder);
        this.gyro = new Pigeon2(config.gyro);
        this.headingController = config.headingController;
        this.kinematics = new DifferentialDriveKinematics(config.ROBOT_WIDTH);

        this.leftLeader.factoryDefault();
        this.leftFollower.factoryDefault();
        this.rightLeader.factoryDefault();
        this.rightFollower.factoryDefault();
        this.leftLeader.setPid(config.pidController);
        this.rightLeader.setPid(config.pidController);
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        this.rightLeader.setInverted(true);
        this.leftFollower.follow(this.leftLeader);
        this.rightFollower.follow(this.rightLeader);

        this.gyro.reset();

        this.leftEncoder.reset();
        this.rightEncoder.reset();

        var wheelPositions = this.getWheelPositions();
        this.odometry = new DifferentialDriveOdometry(this.getYaw(), wheelPositions.leftMeters,
                wheelPositions.rightMeters);
    }

    private DifferentialDriveWheelPositions getWheelPositions() {
        return new DifferentialDriveWheelPositions(leftEncoder.getRadians() * WHEEL_RADIUS,
                rightEncoder.getRadians() * WHEEL_RADIUS);
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
        this.setSpeeds(kinematics.toWheelSpeeds(chassisSpeeds));
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        leftLeader.velocity(speeds.leftMetersPerSecond * GEAR_RATIO / WHEEL_RADIUS);
        rightLeader.velocity(speeds.rightMetersPerSecond * GEAR_RATIO / WHEEL_RADIUS);
    }

    public Command driveCommand(Supplier<ChassisSpeeds> directAngle, Supplier<ChassisSpeeds> angularVelocity, boolean driveDirectAngle) {
        return run(() -> this.drive(driveDirectAngle ? directAngle.get() : angularVelocity.get()));
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        this.odometry.update(this.getYaw(), this.getWheelPositions());
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Differential Drivetrain");
        builder.setActuator(true);
        builder.addBooleanProperty("Direct Angle", this::getDirectAngle, this::setDirectAngle);
        builder.addDoubleProperty("Heading", () -> -this.getYaw().getDegrees(), null);
        builder.addStringProperty("Pose", () -> this.getPose().getTranslation().toString(), null);

        SmartDashboard.putData("Reset Heading", Commands.runOnce(this::zeroHeading));
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

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        this.odometry.resetPosition(this.getYaw(), this.getWheelPositions(), pose);
    }

    public void setMotorBrake(boolean brake) {
        if (brake) {
            this.leftLeader.brake();
            this.leftFollower.brake();
            this.rightLeader.brake();
            this.rightFollower.brake();
        } else {
            this.leftLeader.shutdown();
            this.leftFollower.shutdown();
            this.rightLeader.shutdown();
            this.rightFollower.shutdown();
        }
    }

    public void toggleDirectAngle() {
        this.directAngle = !this.directAngle;
    }

    private double calculateRotation(Rotation2dSupplier headingSupplier) {
        targetHeading = headingSupplier.asTranslation().getNorm() < 0.5 ? targetHeading : headingSupplier.get();
        return MathUtil.clamp(headingController.calculate(this.getYaw().minus(targetHeading).getRadians(), 0), -1, 1);
    }

    public ChassisSpeeds getChassisSpeeds(double velocity, double rotation) {
        return new ChassisSpeeds(velocity * MAX_SPEED, 0, rotation * MAX_ANGULAR_SPEED);
    }

    public static class Config {
        public double ROBOT_WIDTH; // meters
        public double MAX_SPEED; // meters per second
        public double MAX_ANGULAR_SPEED; // one rotation per second
        public double GEAR_RATIO;
        public double WHEEL_RADIUS; // meters
        public int leftLeader;
        public int leftFollower;
        public int rightLeader;
        public int rightFollower;
        public int leftEncoder;
        public int rightEncoder;
        public int gyro;
        public PIDController pidController;
        public PIDController headingController;
    }

    public static class DifferentialInputStream implements Supplier<ChassisSpeeds> {
        private final ZDifferential drivetrain;
        private final DoubleSupplier velocity;
        private double deadband = 0;
        private RotationType rotationType = RotationType.NONE;
        private Rotation2dSupplier heading;
        private DoubleSupplier rotation;

        public DifferentialInputStream(ZDifferential drivetrain, DoubleSupplier velocity) {
            this.drivetrain = drivetrain;
            this.velocity = velocity;
        }

        public DifferentialInputStream withRotation(DoubleSupplier rotation) {
            this.rotation = rotation;
            this.rotationType = DifferentialInputStream.RotationType.ROTATION;
            return this;
        }

        public DifferentialInputStream deadband(double deadband) {
            this.deadband = deadband;
            return this;
        }

        public DifferentialInputStream withHeading(Rotation2dSupplier heading) {
            this.heading = heading;
            this.rotationType = DifferentialInputStream.RotationType.HEADING;
            return this;
        }

        @Override
        public ChassisSpeeds get() {
            var velocity = Maths.cube(MathUtil.applyDeadband(this.velocity.getAsDouble(), this.deadband));
            var rotation = switch (rotationType) {
                case HEADING -> this.drivetrain.calculateRotation(heading);
                case ROTATION -> Maths.cube(MathUtil.applyDeadband(this.rotation.getAsDouble(), this.deadband));
                case NONE -> 0;
            };
            return drivetrain.getChassisSpeeds(velocity, rotation);
        }

        enum RotationType {
            HEADING, ROTATION, NONE
        }
    }
}
