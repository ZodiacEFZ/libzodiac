package frc.libzodiac.hardware.group;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import frc.libzodiac.api.SwerveModule;
import frc.libzodiac.drivetrain.Swerve;
import frc.libzodiac.hardware.MagEncoder;
import frc.libzodiac.hardware.TalonFXMotor;

public class TalonFXSwerveModule implements SwerveModule {
    private final double WHEEL_RADIUS;
    private final TalonFXMotor angle;
    private final TalonFXMotor drive;
    private final MagEncoder encoder;
    private Rotation2d lastAngle;

    public TalonFXSwerveModule(Config config, Swerve.Config swerveConfig) {
        this.drive = new TalonFXMotor(config.drive);
        this.angle = new TalonFXMotor(config.angle);
        this.angle.factoryDefault();
        this.drive.factoryDefault();
        this.angle.setPID(config.anglePID != null ? config.anglePID : swerveConfig.anglePID);
        this.drive.setPID(config.drivePID != null ? config.drivePID : swerveConfig.drivePID);
        this.angle.setInverted(config.angleReversed);
        this.drive.setInverted(config.driveReversed);
        this.angle.setSensorToMechanismRatio(swerveConfig.ANGLE_GEAR_RATIO);
        this.drive.setSensorToMechanismRatio(swerveConfig.DRIVE_GEAR_RATIO);
        this.angle.setContinuous(true);
        this.angle.setBrakeWhenNeutral(true);
        this.drive.setBrakeWhenNeutral(false);

        this.encoder = new MagEncoder(config.encoder, config.encoderZero);
        this.lastAngle = this.getAngle();

        this.WHEEL_RADIUS = swerveConfig.WHEEL_RADIUS;
    }

    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d angle) {
        var delta = desiredState.angle.minus(angle);
        if (Math.abs(new Rotation2d(delta.getCos(), delta.getSin()).getRadians()) > Math.PI / 2) {
            return new SwerveModuleState(-desiredState.speedMetersPerSecond,
                    desiredState.angle.plus(new Rotation2d(Math.PI)));
        } else {
            return desiredState;
        }
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
        var currentAngle = this.getAngle();
        // Optimize the reference state to avoid spinning further than 90 degrees
        var optimizedDesiredState = optimize(desired, currentAngle);
        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        optimizedDesiredState.cosineScale(currentAngle);

        this.drive.velocity(Units.RadiansPerSecond.of(optimizedDesiredState.speedMetersPerSecond / this.WHEEL_RADIUS));

        if (Math.abs(optimizedDesiredState.speedMetersPerSecond) >= 0.03 && Math.abs(this.lastAngle.minus(optimizedDesiredState.angle).getRadians()) >= Math.PI / 60) {
            this.lastAngle = optimizedDesiredState.angle;
        }
        this.angle.position(this.lastAngle.getMeasure());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.drive.getVelocity().in(Units.RadiansPerSecond) * this.WHEEL_RADIUS,
                this.getAngle());
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.drive.getPosition().in(Units.Radians) * this.WHEEL_RADIUS,
                this.getAngle());
    }

    public TalonFXMotor getAngleMotor() {
        return this.angle;
    }

    public TalonFXMotor getDriveMotor() {
        return this.drive;
    }

    public static class Config {
        int angle;
        int drive;
        int encoder;
        int encoderZero;
        boolean angleReversed = false;
        boolean driveReversed = false;
        PIDController drivePID = null;
        PIDController anglePID = null;

        public Config(int angle, int drive, int encoder, int encoderZero) {
            this.angle = angle;
            this.drive = drive;
            this.encoder = encoder;
            this.encoderZero = encoderZero;
        }

        public Config(int angle, int drive, int encoder, int encoderZero, boolean angleReversed, boolean driveReversed) {
            this(angle, drive, encoder, encoderZero);
            this.angleReversed = angleReversed;
            this.driveReversed = driveReversed;
        }

        public Config(int angle, int drive, int encoder, int encoderZero, boolean angleReversed, boolean driveReversed, PIDController drivePID, PIDController anglePID) {
            this(angle, drive, encoder, encoderZero, angleReversed, driveReversed);
            this.drivePID = drivePID;
            this.anglePID = anglePID;
        }
    }
}
