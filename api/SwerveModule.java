package frc.libzodiac.api;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    void shutdown();

    default void brake() {
        throw new UnsupportedOperationException();
    }

    /**
     * Set the state of which the swerve module to drive to.
     *
     * @param desired desired state with speed and angle
     */
    void setDesiredState(SwerveModuleState desired);

    /**
     * Returns the current state of the module.
     *
     * @return the state
     * @throws UnsupportedOperationException if not implemented
     */
    default SwerveModuleState getState() {
        throw new UnsupportedOperationException();
    }

    /**
     * Returns the current position of the module.
     *
     * @return the position
     * @throws UnsupportedOperationException if not implemented
     */
    default SwerveModulePosition getPosition() {
        throw new UnsupportedOperationException();
    }
}
