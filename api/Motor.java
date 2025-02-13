package frc.libzodiac.api;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Defines a large collection of APIs to operate various motors so that motors
 * can be controlled under a unified generic way.
 */
public interface Motor {
    void setInverted(boolean inverted);

    void invert();

    /**
     * Stop any output behaviour of this motor.
     */
    void shutdown();

    /**
     * Brake the motor's motion.
     *
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void brake() {
        throw new UnsupportedOperationException("the motor does not support braking");
    }

    /**
     * Control the motor by percent power output.
     *
     * @param percent the percent power in [-1,1]
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void power(double percent) {
        throw new UnsupportedOperationException("the motor does not support output by percent power");
    }

    /**
     * Control the motor to turn to a specific position.
     *
     * @param position the target position
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     * @apiNote Zero position is implementation defined.
     */
    default void position(Angle position) {
        throw new UnsupportedOperationException("the motor does not support turning by position");
    }

    /**
     * Control the motor by its output angular velocity.
     *
     * @param angularVelocity the angular velocity
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void velocity(AngularVelocity angularVelocity) {
        throw new UnsupportedOperationException("the motor does not support output by angular velocity");
    }

    /**
     * Control the motor by its output voltage.
     *
     * @param voltage the voltage
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void voltage(Voltage voltage) {
        throw new UnsupportedOperationException("the motor does not support output by voltage");
    }

    /**
     * Control the motor by its output current.
     *
     * @param current the current
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void current(Current current) {
        throw new UnsupportedOperationException("the motor does not support output by current");
    }
}
