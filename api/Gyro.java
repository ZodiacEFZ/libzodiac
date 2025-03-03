package frc.libzodiac.api;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Generic interface for gyroscopes.
 */
public interface Gyro {

    /**
     * Reset the gyroscope, using current direction as zero position.
     */
    void reset();

    /**
     * Get the heading angle of robot.
     *
     * @return the measured {@link Rotation2d}
     */
    Rotation2d getRotation2d();

    /**
     * Get the 3D orientation (yaw, pitch and roll) of robot.
     *
     * @return the measured {@link Rotation3d}
     */
    default Rotation3d getRotation3d() {
        throw new UnsupportedOperationException();
    }

    default AngularVelocity getYawAngularVelocity() {
        throw new UnsupportedOperationException();
    }

    default AngularVelocity getPitchAngularVelocity() {
        throw new UnsupportedOperationException();
    }

    default AngularVelocity getRollAngularVelocity() {
        throw new UnsupportedOperationException();
    }

}
