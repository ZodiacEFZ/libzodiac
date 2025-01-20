package frc.libzodiac.unused.api;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.libzodiac.unused.util.Vec2;

/**
 * The general interface to access a swerve drive module.
 */
public interface Zwerve {

    /**
     * Set the velocity vector output of the swerve module.
     *
     * @param vel the velocity vector
     */
    void set_vel(Vec2 vel);

    void shutdown();

    /**
     * Retrieve the odometry information from the swerve module.
     *
     * @return current odometry information
     */
    SwerveModulePosition odo();

}
