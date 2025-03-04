package frc.libzodiac.hardware.group;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.libzodiac.api.SwerveModule;
import frc.libzodiac.drivetrain.Swerve;
import frc.libzodiac.hardware.MagEncoder;
import frc.libzodiac.hardware.TalonFXMotor;

public class TalonFXSwerveModule implements SwerveModule {
    private final Distance     wheelRadius;
    private final TalonFXMotor angle;
    private final TalonFXMotor drive;
    private final MagEncoder   encoder;
    private       Rotation2d   lastAngle;

    public TalonFXSwerveModule(Config config, Swerve.Config parent) {
        this.drive = new TalonFXMotor(config.drive);
        this.angle = new TalonFXMotor(config.angle);
        this.angle.factoryDefault();
        this.drive.factoryDefault();
        this.angle.setPID(config.anglePID != null ? config.anglePID : parent.anglePID);
        this.drive.setPID(config.drivePID != null ? config.drivePID : parent.drivePID);
        this.angle.setInverted(config.angleInverted);
        this.drive.setInverted(config.driveInverted);
        this.angle.setSensorToMechanismRatio(parent.angleGearRatio);
        this.drive.setSensorToMechanismRatio(parent.driveGearRatio);
        this.angle.setContinuous(true);
        this.angle.setBrakeWhenNeutral(true);
        this.drive.setBrakeWhenNeutral(false);

        this.encoder   = new MagEncoder(config.encoder, config.encoderZero);
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
            Math.abs(this.lastAngle.minus(optimizedDesiredState.angle).getRadians()) >= Math.PI / 60) {
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
        return new SwerveModuleState(this.drive.getVelocity().asFrequency().times(this.wheelRadius.times(2 * Math.PI)),
                                     this.getAngle());
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.wheelRadius.times(this.drive.getPosition().in(Units.Radians)),
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
        public int           angle;
        /**
         * CAN ID of drive motor.
         */
        public int           drive;
        /**
         * CAN ID of absolute encoder.
         */
        public int           encoder;
        /**
         * Zero position of encoder in raw units.
         */
        public double        encoderZero;
        /**
         * Reversion state of angle motor.
         */
        public boolean       angleInverted = false;
        /**
         * Reversion state of drive motor.
         */
        public boolean       driveInverted = false;
        /**
         * PID arguments for drive motor.
         */
        public PIDController drivePID      = null;
        /**
         * PID arguments for angle motor.
         */
        public PIDController anglePID      = null;

        public Config withEncoderZero(double encoderZero) {
            this.encoderZero = encoderZero;
            return this;
        }

        public Config withAngleInverted(boolean angleInverted) {
            this.angleInverted = angleInverted;
            return this;
        }

        public Config withDriveInverted(boolean driveInverted) {
            this.driveInverted = driveInverted;
            return this;
        }

        public TalonFXSwerveModule build(Swerve.Config parent) {
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

        public Config withEncoder(int encoder) {
            this.encoder = encoder;
            return this;
        }

        public Config withDrivePID(PIDController drivePID) {
            this.drivePID = drivePID;
            return this;
        }

        public Config withAnglePID(PIDController anglePID) {
            this.anglePID = anglePID;
            return this;
        }
    }
}
