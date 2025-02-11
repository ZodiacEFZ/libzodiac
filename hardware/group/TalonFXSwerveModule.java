package frc.libzodiac.hardware.group;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.libzodiac.api.ZwerveModule;
import frc.libzodiac.drivetrain.Zwerve;
import frc.libzodiac.hardware.MagEncoder;
import frc.libzodiac.hardware.TalonFXMotor;

public class TalonFXSwerveModule implements Sendable, ZwerveModule {
    private final double ANGLE_GEAR_RATIO;
    private final double DRIVE_GEAR_RATIO;
    private final double WHEEL_RADIUS;
    private final TalonFXMotor angle;
    private final TalonFXMotor drive;
    private final MagEncoder encoder;
    private Rotation2d lastAngle;

    public TalonFXSwerveModule(Config config, Zwerve.Config swerveConfig) {
        this.drive = new TalonFXMotor(config.drive);
        this.angle = new TalonFXMotor(config.angle);
        this.angle.factoryDefault();
        this.drive.factoryDefault();
        this.angle.setPid(swerveConfig.anglePid);
        this.drive.setPid(swerveConfig.drivePid);
        this.angle.setInverted(config.angleReversed);
        this.drive.setInverted(config.driveReversed);
        this.encoder = new MagEncoder(config.encoder, config.encoderZero);
        this.lastAngle = this.getAngle();
        this.ANGLE_GEAR_RATIO = swerveConfig.ANGLE_GEAR_RATIO;
        this.DRIVE_GEAR_RATIO = swerveConfig.DRIVE_GEAR_RATIO;
        this.WHEEL_RADIUS = swerveConfig.WHEEL_RADIUS;
    }

    private Rotation2d getAngle() {
        return new Rotation2d(this.angle.getPosition() / this.ANGLE_GEAR_RATIO);
    }

    /**
     * Zeroes all the SwerveModule encoders.
     */
    public void resetEncoder() {
        this.angle.setPosition(this.getAngleEncoder().getRadians() * this.ANGLE_GEAR_RATIO);
    }

    private Rotation2d getAngleEncoder() {
        return this.encoder.getRotation2d();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.drive.getVelocity() / this.DRIVE_GEAR_RATIO * this.WHEEL_RADIUS,
                this.getAngle());
    }

    @Override
    public void shutdown() {
        this.drive.shutdown();
    }

    @Override
    public void brake() {
        this.drive.brake();
    }

    @Override
    public void setDesiredState(SwerveModuleState desired) {
        var currentAngle = this.getAngle();
        // Optimize the reference state to avoid spinning further than 90 degrees
        var optimizedDesiredState = optimize(desired, currentAngle);
        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        optimizedDesiredState.cosineScale(currentAngle);

        this.drive.velocity(optimizedDesiredState.speedMetersPerSecond / this.WHEEL_RADIUS * this.DRIVE_GEAR_RATIO);

        this.lastAngle = (Math.abs(
                optimizedDesiredState.speedMetersPerSecond) < 0.03) ? this.lastAngle : optimizedDesiredState.angle;
        this.angle.angle(this.lastAngle.getRadians() * this.ANGLE_GEAR_RATIO);
    }

    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d angle) {
        var currentAngle = new Rotation2d(angle.getCos(), angle.getSin());
        var desiredAngle = new Rotation2d(desiredState.angle.getCos(), desiredState.angle.getSin());
        var targetSpeed = desiredState.speedMetersPerSecond;
        var delta = desiredAngle.minus(currentAngle);
        var targetAngle = angle.plus(delta);
        if (Math.abs(delta.getRadians()) > Math.PI / 2) {
            targetSpeed = -targetSpeed;
            targetAngle = targetAngle.minus(new Rotation2d(Math.PI));
        }
        return new SwerveModuleState(targetSpeed, targetAngle);
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.drive.getPosition() / this.DRIVE_GEAR_RATIO * this.WHEEL_RADIUS,
                this.getAngle());
    }

    public void setMotorBrake(boolean brake) {
        if (brake) {
            this.drive.brake();
        } else {
            this.drive.shutdown();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module");
        builder.addDoubleProperty("Angle", () -> {
            var angle = this.getAngle();
            return new Rotation2d(angle.getCos(), angle.getSin()).getRadians();
        }, null);
        builder.addDoubleProperty("Speed", this.drive::getVelocity, null);
    }

    public TalonFXMotor getAngleMotor() {
        return this.angle;
    }

    public TalonFXMotor getDriveMotor() {
        return this.drive;
    }

    public static class Config {
        final int angle;
        final int drive;
        final int encoder;
        final int encoderZero;
        final boolean angleReversed;
        final boolean driveReversed;

        public Config(int angle, int drive, int encoder, int encoderZero, boolean angleReversed,
                      boolean driveReversed) {
            this.angle = angle;
            this.drive = drive;
            this.encoder = encoder;
            this.encoderZero = encoderZero;
            this.angleReversed = angleReversed;
            this.driveReversed = driveReversed;
        }
    }
}
