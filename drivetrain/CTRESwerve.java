package frc.libzodiac.drivetrain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.libzodiac.api.SwerveDrivetrain;
import frc.libzodiac.hardware.group.CommandCTRESwerve;
import frc.libzodiac.util.Maths;
import frc.libzodiac.util.Rotation2dSupplier;

import java.util.Arrays;
import java.util.Collection;
import java.util.Optional;

public class CTRESwerve extends CommandCTRESwerve implements SwerveDrivetrain {
    private static final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric().withDriveRequestType(
            SwerveModule.DriveRequestType.Velocity);
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric().withDriveRequestType(
            SwerveModule.DriveRequestType.Velocity);
    private final PathConstraints constraints;
    private final Field2d field = new Field2d();
    private final PIDController headingPID;
    private boolean fieldCentric = true;
    private boolean directAngle = true;
    private boolean slowMode = false;
    private Rotation2d targetHeading = new Rotation2d();

    public CTRESwerve(PathConstraints constraints, PIDController headingPID,
                      SwerveDrivetrainConstants drivetrainConstants,
                      SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        this.constraints = constraints;
        this.headingPID = headingPID;
        this.field.setRobotPose(this.getPose());
    }

    @Override
    public Pose2d getPose() {
        return this.getState().Pose;
    }

    @Override
    public void setPose(Pose2d pose2d) {
        this.resetPose(pose2d);
    }

    @Override
    public ChassisSpeeds getRobotCentricSpeeds() {
        return this.getState().Speeds;
    }

    @Override
    public void driveRobotCentric(ChassisSpeeds chassisSpeeds) {
        var states = PathPlanner.generateSwerveSetpoint(chassisSpeeds);
        if (states == null) {
            this.setControl(this.driveRobotCentric.withVelocityX(chassisSpeeds.vxMetersPerSecond)
                                                  .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                                                  .withRotationalRate(
                                                          chassisSpeeds.omegaRadiansPerSecond));
        } else {
            var speeds = states.robotRelativeSpeeds();
            this.setControl(this.driveRobotCentric.withVelocityX(speeds.vxMetersPerSecond)
                                                  .withVelocityY(speeds.vyMetersPerSecond)
                                                  .withRotationalRate(
                                                          speeds.omegaRadiansPerSecond));
        }
    }

    @Override
    public PathFollowingController getPathFollowingController() {
        return new PPHolonomicDriveController(
                // Translation PID constants
                new PIDConstants(10, 0, 0, 1),
                // Rotation PID constants
                new PIDConstants(this.headingPID.getP(), this.headingPID.getI(),
                                 this.headingPID.getD()));
    }

    @Override
    public Field2d getField() {
        return this.field;
    }

    @Override
    public double getMaxAngularVelocity() {
        return this.constraints.maxAngularVelocityRadPerSec();
    }

    @Override
    public Optional<SwerveModuleState[]> getModuleStates() {
        return Optional.of(this.getState().ModuleStates);
    }

    @Override
    public void shutdown() {
        this.setControl(idle);
    }

    @Override
    public void brake() {
        this.setControl(brake);
    }

    @Override
    public AngularVelocity getAngularVelocity() {
        return this.getPigeon2().getAngularVelocityZWorld().getValue();
    }

    private Rotation2d getGyroYaw() {
        return this.getState().RawHeading;
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
                                            () -> this.getState().ModuleStates[0].angle.getRadians(),
                                            null);
            swerveBuilder.addDoubleProperty("Front Left Velocity",
                                            () -> this.getState().ModuleStates[0].speedMetersPerSecond,
                                            null);

            swerveBuilder.addDoubleProperty("Front Right Angle",
                                            () -> this.getState().ModuleStates[1].angle.getRadians(),
                                            null);
            swerveBuilder.addDoubleProperty("Front Right Velocity",
                                            () -> this.getState().ModuleStates[1].speedMetersPerSecond,
                                            null);

            swerveBuilder.addDoubleProperty("Back Left Angle",
                                            () -> this.getState().ModuleStates[2].angle.getRadians(),
                                            null);
            swerveBuilder.addDoubleProperty("Back Left Velocity",
                                            () -> this.getState().ModuleStates[2].speedMetersPerSecond,
                                            null);

            swerveBuilder.addDoubleProperty("Back Right Angle",
                                            () -> this.getState().ModuleStates[3].angle.getRadians(),
                                            null);
            swerveBuilder.addDoubleProperty("Back Right Velocity",
                                            () -> this.getState().ModuleStates[3].speedMetersPerSecond,
                                            null);

            swerveBuilder.addDoubleProperty("Robot Angle", () -> this.getYawRelative().getRadians(),
                                            null);
        });
        SmartDashboard.putData("Reset Heading",
                               Commands.runOnce(this::zeroHeading).ignoringDisable(true));
    }

    @Override
    public void zeroHeading() {
        this.seedFieldCentric();
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
        this.directAngle = directAngle;
    }

    @Override
    public void toggleDirectAngle() {
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
                                  .times((this.slowMode ? this.constraints.maxVelocity().div(3) :
                                                  this.constraints.maxVelocity()).in(
                                          Units.MetersPerSecond));
        return new ChassisSpeeds(velocity.getX(), velocity.getY(), rotation *
                                                                   this.constraints.maxAngularVelocity()
                                                                                   .in(Units.RadiansPerSecond));
    }

    @Override
    public double calculateRotation(Rotation2dSupplier headingSupplier) {
        this.targetHeading = headingSupplier.asTranslation().getNorm() < 0.5 ? this.targetHeading :
                                     headingSupplier.get();
        return MathUtil.applyDeadband(MathUtil.clamp(this.headingPID.calculate(
                this.getYawRelative().minus(this.targetHeading).getRadians(), 0), -1, 1), 0.02);
    }

    @Override
    public Collection<TalonFX> getTalonFXMotors() {
        return Arrays.stream(this.getModules()).mapMulti((module, consumer) -> {
            consumer.accept(module.getDriveMotor());
            consumer.accept(module.getSteerMotor());
        }).map((motor) -> (TalonFX) motor).toList();
    }

    @Override
    public void setTargetHeading(Rotation2d targetHeading) {
        this.targetHeading = targetHeading;
    }

    @Override
    public void driveFieldCentric(ChassisSpeeds speeds) {
        var robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                                                                        this.getYawRelative());
        var states = PathPlanner.generateSwerveSetpoint(robotRelativeSpeeds);
        if (states == null) {
            this.setControl(
                    this.driveRobotCentric.withVelocityX(robotRelativeSpeeds.vxMetersPerSecond)
                                          .withVelocityY(robotRelativeSpeeds.vyMetersPerSecond)
                                          .withRotationalRate(
                                                  robotRelativeSpeeds.omegaRadiansPerSecond));
        } else {
            this.setControl(this.driveFieldCentric.withVelocityX(speeds.vxMetersPerSecond)
                                                  .withVelocityY(speeds.vyMetersPerSecond)
                                                  .withRotationalRate(
                                                          speeds.omegaRadiansPerSecond));
        }
    }
}

