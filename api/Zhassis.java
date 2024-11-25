package frc.libzodiac.api;

import frc.libzodiac.util.Vec2;

/**
 * The interface to generally operate a chassis.
 */
public interface Zhassis {

    /**
     * Set the translational velocity of the chassis. Use +x axis for ahead.
     */
    void vel(Vec2 vel);

    /**
     * Set the rotational velocity of the chassis. Use positive for
     * counter-clockwise.
     */
    double rot();

    /**
     * Request the chassis to coast.
     */
    void coast();

    /**
     * Request the chassis to brake.
     */
    void brake();

    /**
     * Request the chassis to shutdown.
     */
    void shutdown();
}
