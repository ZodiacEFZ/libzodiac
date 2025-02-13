package frc.libzodiac.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.api.Drivetrain;
import frc.libzodiac.hardware.Limelight;
import frc.libzodiac.hardware.group.TalonFXSwerveModule;
import frc.libzodiac.util.Maths;
import frc.libzodiac.util.Rotation2dSupplier;
import frc.libzodiac.util.Translation2dSupplier;

import java.util.Collection;
import java.util.HashSet;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Zwerve extends SubsystemBase implements Drivetrain {
    // Distance between centers of right and left wheels on robot
    public final double ROBOT_WIDTH;
    // Distance between front and back wheels on robot
    public final double ROBOT_LENGTH;
    public final double MAX_SPEED;
    private final double MAX_ANGULAR_VELOCITY;

    // Robot swerve modules
    private final TalonFXSwerveModule frontLeft;
    private final TalonFXSwerveModule rearLeft;
    private final TalonFXSwerveModule frontRight;
    private final TalonFXSwerveModule rearRight;
    // The gyro sensor
    private final Pigeon2 gyro;

    private final PIDController headingController;
    private final SwerveDriveKinematics kinematics;
    private final Field2d field = new Field2d();
    private final SwerveDrivePoseEstimator poseEstimator;
    private boolean fieldCentric = true;
    private boolean directAngle = true;
    private Rotation2d targetHeading = new Rotation2d();

    /**
     * Creates a new DriveSubsystem.
     */
    public Zwerve(Config config, Pose2d initialPose) {
        this.ROBOT_WIDTH = config.ROBOT_WIDTH;
        this.ROBOT_LENGTH = config.ROBOT_LENGTH;
        this.MAX_SPEED = config.MAX_SPEED;
        this.MAX_ANGULAR_VELOCITY = config.MAX_ANGULAR_VELOCITY;
        this.frontLeft = new TalonFXSwerveModule(config.frontLeft, config);
        this.frontRight = new TalonFXSwerveModule(config.frontRight, config);
        this.rearLeft = new TalonFXSwerveModule(config.rearLeft, config);
        this.rearRight = new TalonFXSwerveModule(config.rearRight, config);
        this.gyro = new Pigeon2(config.gyro);
        this.headingController = config.headingController;

        this.kinematics = new SwerveDriveKinematics(new Translation2d(this.ROBOT_LENGTH / 2, this.ROBOT_WIDTH / 2),
                new Translation2d(this.ROBOT_LENGTH / 2, -this.ROBOT_WIDTH / 2),
                new Translation2d(-this.ROBOT_LENGTH / 2, this.ROBOT_WIDTH / 2),
                new Translation2d(-this.ROBOT_LENGTH / 2, -this.ROBOT_WIDTH / 2));

        this.gyro.getConfigurator().apply(new Pigeon2Configuration());
        this.zeroHeading();

        // By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
        Timer.delay(0.5);
        this.resetEncoders();
        Timer.delay(0.5);

        this.poseEstimator = new SwerveDrivePoseEstimator(this.kinematics, this.getYaw(), this.getModulePositions(),
                initialPose);
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
        this.frontLeft.resetEncoder();
        this.rearLeft.resetEncoder();
        this.frontRight.resetEncoder();
        this.rearRight.resetEncoder();
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
        this.poseEstimator.update(this.getYaw(), this.getModulePositions());
        Limelight.update();
        this.field.setRobotPose(this.getPose());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Drivetrain");
        builder.setActuator(true);
        builder.addBooleanProperty("Field Centric", this::getFieldCentric, this::setFieldCentric);
        builder.addBooleanProperty("Direct Angle", this::getDirectAngle, this::setDirectAngle);
        builder.addDoubleProperty("Heading", () -> -this.getYaw().getDegrees(), null);
        builder.addStringProperty("Pose", () -> this.getPose().getTranslation().toString(), null);

        SmartDashboard.putData("Front Left Module", this.frontLeft);
        SmartDashboard.putData("Front Right Module", this.frontRight);
        SmartDashboard.putData("Rear Left Module", this.rearLeft);
        SmartDashboard.putData("Rear Right Module", this.rearRight);

        SmartDashboard.putData("Reset Heading", Commands.runOnce(this::zeroHeading));
    }

    public boolean getFieldCentric() {
        return this.fieldCentric;
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public boolean getDirectAngle() {
        return this.directAngle;
    }

    public void setDirectAngle(boolean directAngle) {
        this.directAngle = directAngle;
    }

    public void driveFieldOriented(ChassisSpeeds speeds) {
        this.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getYaw()));
    }

    public ChassisSpeeds calculateChassisSpeeds(Translation2d translation, double rotation) {
        translation = translation.times(this.MAX_SPEED);
        rotation = rotation * this.MAX_ANGULAR_VELOCITY;
        return new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        this.drive(this.calculateChassisSpeeds(translation, rotation), fieldRelative);
    }

    private void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        SmartDashboard.putString("Chassis Speeds", chassisSpeeds.toString());
        if (fieldRelative) {
            this.driveFieldOriented(chassisSpeeds);
        } else {
            this.driveRobotRelative(chassisSpeeds);
        }
    }

    public Command getDriveCommand(Supplier<ChassisSpeeds> directAngle, Supplier<ChassisSpeeds> angularVelocity, BooleanSupplier driveDirectAngle, BooleanSupplier fieldRelative) {
        return run(() -> this.drive(driveDirectAngle.getAsBoolean() ? directAngle.get() : angularVelocity.get(),
                fieldRelative.getAsBoolean()));
    }

    public void setMotorBrake(boolean brake) {
        this.frontLeft.setMotorBrake(brake);
        this.frontRight.setMotorBrake(brake);
        this.rearLeft.setMotorBrake(brake);
        this.rearRight.setMotorBrake(brake);
    }

    public void toggleFieldCentric() {
        this.fieldCentric = !this.fieldCentric;
    }

    public void centerModules() {
        this.frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
        this.frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
        this.rearLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
        this.rearRight.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
    }

    private double calculateRotation(Rotation2dSupplier headingSupplier) {
        this.targetHeading = headingSupplier.asTranslation()
                .getNorm() < 0.5 ? this.targetHeading : headingSupplier.get();
        return MathUtil.clamp(this.headingController.calculate(this.getYaw().minus(this.targetHeading).getRadians(), 0),
                -1, 1);
    }

    public void toggleDirectAngle() {
        this.directAngle = !this.directAngle;
    }

    public Collection<ParentDevice> getMotors() {
        Collection<ParentDevice> motors = new HashSet<>();
        motors.add(this.frontLeft.getAngleMotor().getRawEntry());
        motors.add(this.frontLeft.getDriveMotor().getRawEntry());
        motors.add(this.frontRight.getAngleMotor().getRawEntry());
        motors.add(this.frontRight.getDriveMotor().getRawEntry());
        motors.add(this.rearLeft.getAngleMotor().getRawEntry());
        motors.add(this.rearLeft.getDriveMotor().getRawEntry());
        motors.add(this.rearRight.getAngleMotor().getRawEntry());
        motors.add(this.rearRight.getDriveMotor().getRawEntry());
        return motors;
    }

    @Override
    public SwerveDrivePoseEstimator getPoseEstimator() {
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
        this.poseEstimator.resetPosition(this.getYaw(), this.getModulePositions(), pose);
    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.kinematics.toChassisSpeeds(this.getModuleStates());
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states;
        states = PathPlanner.generateSwerveSetpoint(speeds);
        if (states == null) {
            states = this.kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, this.MAX_SPEED);
        }
        this.setModuleStates(states);
    }

    @Override
    public Subsystem getSubsystemBase() {
        return this;
    }

    @Override
    public PPHolonomicDriveController getPathFollowingController() {
        //TODO: Tune these PID constants
        return new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(this.headingController.getP(), this.headingController.getI(),
                        this.headingController.getD())// Rotation PID constants
        );
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
        return new SwerveModuleState[]{this.frontLeft.getState(), this.frontRight.getState(), this.rearLeft.getState(),
                this.rearRight.getState()};
    }

    @Override
    public boolean isSwerve() {
        return true;
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

    public static class Config {
        // Distance between centers of right and left wheels on robot
        public double ROBOT_WIDTH;
        // Distance between front and back wheels on robot
        public double ROBOT_LENGTH;
        public double MAX_SPEED;
        public double MAX_ANGULAR_VELOCITY;

        // Robot swerve modules
        public TalonFXSwerveModule.Config frontLeft;
        public TalonFXSwerveModule.Config rearLeft;
        public TalonFXSwerveModule.Config frontRight;
        public TalonFXSwerveModule.Config rearRight;
        // The gyro sensor
        public int gyro;

        public PIDController headingController;

        public double ANGLE_GEAR_RATIO;
        public double DRIVE_GEAR_RATIO;
        public double WHEEL_RADIUS;

        public PIDController drivePid;
        public PIDController anglePid;
    }

    public static class InputStream implements Supplier<ChassisSpeeds> {
        private final Zwerve drivetrain;
        private final Translation2dSupplier translation;
        private double deadband = 0;
        private RotationType rotationType = RotationType.NONE;
        private Rotation2dSupplier heading;
        private DoubleSupplier rotation;

        public InputStream(Zwerve drivetrain, Translation2dSupplier translation) {
            this.drivetrain = drivetrain;
            this.translation = translation;
        }

        @Override
        public ChassisSpeeds get() {
            var translation = Maths.cubeTranslation(Maths.applyDeadband(this.translation.get(), this.deadband));
            var rotation = switch (this.rotationType) {
                case HEADING -> this.drivetrain.calculateRotation(this.heading);
                case ROTATION -> Maths.cube(MathUtil.applyDeadband(this.rotation.getAsDouble(), this.deadband));
                case NONE -> 0;
            };
            return this.drivetrain.calculateChassisSpeeds(translation, rotation);
        }

        public InputStream rotation(DoubleSupplier rotation) {
            this.rotation = rotation;
            this.rotationType = RotationType.ROTATION;
            return this;
        }

        public InputStream deadband(double deadband) {
            this.deadband = deadband;
            return this;
        }

        public InputStream heading(Rotation2dSupplier heading) {
            this.heading = heading;
            this.rotationType = RotationType.HEADING;
            return this;
        }

        enum RotationType {
            HEADING, ROTATION, NONE
        }
    }
}
