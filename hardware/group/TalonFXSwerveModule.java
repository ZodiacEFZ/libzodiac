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
    private final Distance wheelRadius;
    private final TalonFXMotor angle;
    private final TalonFXMotor drive;
    private final MagEncoder encoder;
    private Rotation2d lastAngle;

    public TalonFXSwerveModule(Config config, Swerve.Config parent) {
        this.drive = new TalonFXMotor(config.drive);
        this.angle = new TalonFXMotor(config.angle);
        this.angle.factoryDefault();
        this.drive.factoryDefault();
        this.angle.setPID(config.anglePID != null ? config.anglePID : parent.anglePID);
        this.drive.setPID(config.drivePID != null ? config.drivePID : parent.drivePID);
        this.angle.setInverted(config.angleReversed);
        this.drive.setInverted(config.driveReversed);
        this.angle.setSensorToMechanismRatio(parent.angleGearRatio);
        this.drive.setSensorToMechanismRatio(parent.driveGearRatio);
        this.angle.setContinuous(true);
        this.angle.setBrakeWhenNeutral(true);
        this.drive.setBrakeWhenNeutral(false);

        this.encoder = new MagEncoder(config.encoder, config.encoderZero);
        this.lastAngle = this.getAngle();

        this.wheelRadius = parent.wheelRadius;
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

        this.drive.velocity(Units.RadiansPerSecond.of(optimizedDesiredState.speedMetersPerSecond / this.wheelRadius.in(Units.Meter)));

        if (Math.abs(optimizedDesiredState.speedMetersPerSecond) >= 0.03 && Math.abs(this.lastAngle
                .minus(optimizedDesiredState.angle).getRadians()) >= Math.PI / 60) {
            this.lastAngle = optimizedDesiredState.angle;
        }
        this.angle.position(this.lastAngle.getMeasure());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.drive.getVelocity()
                                               .in(Units.RadiansPerSecond) * this.wheelRadius.in(Units.Meter),
                this.getAngle());
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.drive.getPosition().in(Units.Radians) * this.wheelRadius.in(Units.Meter),
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
        int angle;
        /**
         * CAN ID of drive motor.
         */
        int drive;
        /**
         * CAN ID of absolute encoder.
         */
        int encoder;
        int encoderZero;
        /**
         * Reversion state of angle motor.
         */
        boolean angleReversed = false;
        /**
         * Reversion state of drive motor.
         */
        boolean driveReversed = false;
        /**
         * PID arguments for drive motor.
         */
        PIDController drivePID = null;
        /**
         * PID arguments for angle motor.
         */
        PIDController anglePID = null;

        public TalonFXSwerveModule build(Swerve.Config parent) {
            return new TalonFXSwerveModule(this, parent);
        }

        public Config withAngleReversed(boolean angleReversed) {
            this.angleReversed = angleReversed;
            return this;
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

        public Config withEncoderZero(int encoderZero) {
            this.encoderZero = encoderZero;
            return this;
        }

        public Config withDriveReversed(boolean driveReversed) {
            this.driveReversed = driveReversed;
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
