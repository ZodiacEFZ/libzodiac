package frc.libzodiac.hardware.group;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.libzodiac.hardware.MagEncoder;
import frc.libzodiac.hardware.TalonFXMotor;

public class TalonFXSwerveModule {
    private static final double ANGLE_GEAR_RATIO = 150.0 / 7.0;
    private static final double DRIVE_GEAR_RATIO = 6.75;
    private static final double WHEEL_RADIUS = 2; //todo
    private final TalonFXMotor angle;
    private final TalonFXMotor drive;
    private final MagEncoder encoder;
    private Rotation2d lastAngle;

    public TalonFXSwerveModule(int angle, int drive, int encoder, int encoderOffset, boolean angleReversed, boolean driveReversed) {
        this.drive = new TalonFXMotor(drive, 0.15, 0, 2);
        this.angle = new TalonFXMotor(angle, 10, 0.5, 0.5);
        this.angle.setInverted(angleReversed);
        this.drive.setInverted(driveReversed);
        this.encoder = new MagEncoder(encoder).setZero(encoderOffset);
        this.lastAngle = this.getAngle();
    }

    private Rotation2d getAngle() {
        return new Rotation2d(this.angle.getPosition() / ANGLE_GEAR_RATIO);
    }

    /**
     * Zeroes all the SwerveModule encoders.
     */
    public void resetEncoder() {
        this.angle.setPosition(this.getAngleEncoder().getRadians() * ANGLE_GEAR_RATIO);
    }

    private Rotation2d getAngleEncoder() {
        return this.encoder.getRotation2d();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.drive.getVelocity() / DRIVE_GEAR_RATIO * WHEEL_RADIUS, this.getAngle());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState = optimize(desiredState, this.getAngle());

        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        desiredState.cosineScale(this.getAngle());

        this.drive.velocity(desiredState.speedMetersPerSecond / WHEEL_RADIUS * DRIVE_GEAR_RATIO);

        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= 0.03) ? this.lastAngle : desiredState.angle;
        this.angle.angle(angle.getRadians() * ANGLE_GEAR_RATIO);
        this.lastAngle = angle;
    }

    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = Units.degreesToRadians(targetAngle - currentAngle.getDegrees());
        if (Math.abs(delta) > Math.PI / 2) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > Math.PI / 2 ? (targetAngle - Math.PI) : (targetAngle + Math.PI);
        }
        return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound, upperBound, lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.drive.getPosition() / DRIVE_GEAR_RATIO * WHEEL_RADIUS, this.getAngle());
    }
}
