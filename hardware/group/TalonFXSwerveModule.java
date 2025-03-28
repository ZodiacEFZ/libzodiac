package frc.libzodiac.hardware.group;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.libzodiac.api.Encoder;
import frc.libzodiac.api.SwerveModule;
import frc.libzodiac.drivetrain.TalonFXSwerve;
import frc.libzodiac.hardware.TalonFXMotor;

public class TalonFXSwerveModule implements SwerveModule {
    private final Distance wheelRadius;
    private final TalonFXMotor angle;
    private final TalonFXMotor drive;
    private final Encoder encoder;
    private Rotation2d lastAngle;

    public TalonFXSwerveModule(Config config, TalonFXSwerve.Config parent) {
        this.drive = new TalonFXMotor(config.drive);
        this.angle = new TalonFXMotor(config.angle);
        this.angle.factoryDefault();
        this.drive.factoryDefault();
        this.angle.applyConfiguration(
                config.angleConfig != null ? config.angleConfig : parent.angleConfig);
        this.drive.applyConfiguration(
                config.driveConfig != null ? config.driveConfig : parent.driveConfig);
        if (parent.angleCurrent != null) {
            this.angle.applyConfiguration(
                    new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
                                              .withStatorCurrentLimit(parent.angleCurrent));
        }
        if (parent.driveCurrent != null) {
            this.drive.applyConfiguration(
                    new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
                                              .withStatorCurrentLimit(parent.driveCurrent));
        }
        this.angle.setInverted(config.angleInverted);
        this.drive.setInverted(config.driveInverted);
        this.angle.setSensorToMechanismRatio(parent.angleGearRatio);
        this.drive.setSensorToMechanismRatio(parent.driveGearRatio);
        this.angle.setContinuous(true);
        this.angle.setBrakeWhenNeutral(true);
        this.drive.setBrakeWhenNeutral(false);

        this.encoder = config.encoder;
        this.lastAngle = this.getAngle();

        this.wheelRadius = parent.wheelRadius;
    }

    private Rotation2d getAngle() {
        return new Rotation2d(this.angle.getPosition());
    }

    /**
     * Zeroes all the SwerveModule encoders.
     */
    public void resetEncoder() {
        this.angle.setRelativeEncoderPosition(this.getAngleFromEncoder().getMeasure());
    }

    private Rotation2d getAngleFromEncoder() {
        return this.encoder.getRotation2d();
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
        final var currentAngle = this.getAngle();
        // Optimize the reference state to avoid spinning further than 90 degrees
        var optimizedDesiredState = optimize(desired, currentAngle);
        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        optimizedDesiredState.cosineScale(currentAngle);

        this.drive.setVelocity(Units.RadiansPerSecond.of(
                optimizedDesiredState.speedMetersPerSecond / this.wheelRadius.in(Units.Meter)));

        if (Math.abs(optimizedDesiredState.speedMetersPerSecond) >= 0.03 &&
            Math.abs(this.lastAngle.minus(optimizedDesiredState.angle).getRadians()) >=
            Math.PI / 60) {
            this.lastAngle = optimizedDesiredState.angle;
        }
        this.angle.setPosition(this.lastAngle.getMeasure());
    }

    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d angle) {
        final var delta = desiredState.angle.minus(angle);
        if (Math.abs(delta.getRadians()) > Math.PI / 2) {
            return new SwerveModuleState(-desiredState.speedMetersPerSecond,
                                         desiredState.angle.plus(new Rotation2d(Math.PI)));
        } else {
            return desiredState;
        }
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                this.drive.getVelocity().asFrequency().times(this.wheelRadius.times(2 * Math.PI)),
                this.getAngle());
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                this.wheelRadius.times(this.drive.getPosition().in(Units.Radians)),
                this.getAngle());
    }

    public TalonFXMotor getAngleMotor() {
        return this.angle;
    }

    public TalonFXMotor getDriveMotor() {
        return this.drive;
    }

    public static class Config {

        /**
         * CAN ID of angle motor.
         */
        public int angle;
        /**
         * CAN ID of drive motor.
         */
        public int drive;
        /**
         * The absolute encoder.
         */
        public Encoder encoder;
        /**
         * Reversion state of angle motor.
         */
        public boolean angleInverted = false;
        /**
         * Reversion state of drive motor.
         */
        public boolean driveInverted = false;
        /**
         * PID arguments for drive motor.
         */
        public Slot0Configs driveConfig = null;
        /**
         * PID arguments for angle motor.
         */
        public Slot0Configs angleConfig = null;

        public Config withAngleInverted(boolean angleInverted) {
            this.angleInverted = angleInverted;
            return this;
        }

        public Config withDriveInverted(boolean driveInverted) {
            this.driveInverted = driveInverted;
            return this;
        }

        public TalonFXSwerveModule build(TalonFXSwerve.Config parent) {
            return new TalonFXSwerveModule(this, parent);
        }

        public Config withAngle(int angle) {
            this.angle = angle;
            return this;
        }

        public Config withDrive(int drive) {
            this.drive = drive;
            return this;
        }

        public Config withEncoder(Encoder encoder) {
            this.encoder = encoder;
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
    }
}
