// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libzodiac.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.group.TalonFXSwerveModule;
import frc.libzodiac.util.Maths;
import frc.libzodiac.util.Rotation2dSupplier;
import frc.libzodiac.util.Translation2dSupplier;

import java.util.Collection;
import java.util.HashSet;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

    private final PIDController headingController;
    private final SwerveDriveKinematics kinematics;
    private final Field2d field = new Field2d();
    // Odometry class for tracking robot pose
    private final SwerveDriveOdometry odometry;

    private boolean fieldCentric = true;
    private boolean directAngle = true;
    private Rotation2d targetHeading = new Rotation2d();

    /**
     * Creates a new DriveSubsystem.
     */
    public Zwerve(Config config) {
        this.ROBOT_WIDTH = config.ROBOT_WIDTH;
        this.ROBOT_LENGTH = config.ROBOT_LENGTH;
        this.MAX_SPEED = config.MAX_SPEED;
        this.MAX_ANGULAR_SPEED = config.MAX_ANGULAR_SPEED;
        this.frontLeft = new TalonFXSwerveModule(config.frontLeft, config);
        this.frontRight = new TalonFXSwerveModule(config.frontRight, config);
        this.rearLeft = new TalonFXSwerveModule(config.rearLeft, config);
        this.rearRight = new TalonFXSwerveModule(config.rearRight, config);
        this.gyro = new Pigeon2(config.gyro);
        this.headingController = config.headingController;

        kinematics = new SwerveDriveKinematics(new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
                new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
                new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
                new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2));

        this.gyro.getConfigurator().apply(new Pigeon2Configuration());
        this.zeroHeading();

        // By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
        Timer.delay(0.5);
        this.resetEncoders();
        Timer.delay(0.5);

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

    public void driveFieldOriented(ChassisSpeeds speeds) {
        this.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw()));
    }

    public ChassisSpeeds getChassisSpeeds(Translation2d translation, double rotation) {
        translation = translation.times(MAX_SPEED);
        rotation = rotation * MAX_ANGULAR_SPEED;
        return new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        this.drive(this.getChassisSpeeds(translation, rotation), fieldRelative);
    }

    private void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        SmartDashboard.putString("Chassis Speeds", chassisSpeeds.toString());
        if (fieldRelative) {
            this.driveFieldOriented(chassisSpeeds);
        } else {
            this.driveRobotRelative(chassisSpeeds);
        }
    }

    public Command driveCommand(Supplier<ChassisSpeeds> directAngle, Supplier<ChassisSpeeds> angularVelocity, boolean driveDirectAngle, boolean fieldRelative) {
        return run(() -> this.drive(driveDirectAngle ? directAngle.get() : angularVelocity.get(), fieldRelative));
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);
        this.setModuleStates(states);
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
        targetHeading = headingSupplier.asTranslation().getNorm() < 0.5 ? targetHeading : headingSupplier.get();
        return MathUtil.clamp(headingController.calculate(this.getYaw().minus(targetHeading).getRadians(), 0), -1, 1);
    }

    public void toggleDirectAngle() {
        this.directAngle = !this.directAngle;
    }

    public Collection<ParentDevice> getMotors() {
        Collection<ParentDevice> motors = new HashSet<>();
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

    public static class Config {
        // Distance between centers of right and left wheels on robot
        public double ROBOT_WIDTH;
        // Distance between front and back wheels on robot
        public double ROBOT_LENGTH;
        public double MAX_SPEED;
        public double MAX_ANGULAR_SPEED;

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

    public static class SwerveInputStream implements Supplier<ChassisSpeeds> {
        private final Zwerve drivetrain;
        private final Translation2dSupplier translation;
        private double deadband = 0;
        private RotationType rotationType = RotationType.NONE;
        private Rotation2dSupplier heading;
        private DoubleSupplier rotation;

        public SwerveInputStream(Zwerve drivetrain, Translation2dSupplier translation) {
            this.drivetrain = drivetrain;
            this.translation = translation;
        }

        @Override
        public ChassisSpeeds get() {
            var translation = Maths.cubeTranslation(Maths.applyDeadband(this.translation.get(), this.deadband));
            var rotation = switch (rotationType) {
                case HEADING -> this.drivetrain.calculateRotation(heading);
                case ROTATION -> Maths.cube(MathUtil.applyDeadband(this.rotation.getAsDouble(), this.deadband));
                case NONE -> 0;
            };
            return drivetrain.getChassisSpeeds(translation, rotation);
        }

        public SwerveInputStream withRotation(DoubleSupplier rotation) {
            this.rotation = rotation;
            this.rotationType = RotationType.ROTATION;
            return this;
        }

        public SwerveInputStream deadband(double deadband) {
            this.deadband = deadband;
            return this;
        }

        public SwerveInputStream withHeading(Rotation2dSupplier heading) {
            this.heading = heading;
            this.rotationType = RotationType.HEADING;
            return this;
        }

        enum RotationType {
            HEADING, ROTATION, NONE
        }
    }
}
