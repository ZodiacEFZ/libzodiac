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
    /**
     * Set the motor to be inverted.
     *
     * @param inverted whether the motor should be inverted
     */
    void setInverted(boolean inverted);

    /**
     * Invert the motor.
     */
    void invert();

    /**
     * Stop any output behaviour of the motor.
     */
    void shutdown();

    /**
     * Brake the motor.
     *
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void brake() {
        throw new UnsupportedOperationException("the motor does not support braking");
    }

    /**
     * Get current output power of the motor.
     *
     * @return percent power in [-1,1]
     * @throws UnsupportedOperationException if the motor does not support retrieving such data
     */
    default double getPower() {
        throw new UnsupportedOperationException();
    }

    /**
     * Control the motor by percent power output.
     *
     * @param percent the percent power in [-1,1]
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void setPower(double percent) {
        throw new UnsupportedOperationException("the motor does not support output by percent power");
    }

    /**
     * Get current position of the motor.
     *
     * @return the angle of rotation
     * @throws UnsupportedOperationException if the motor does not support retrieving such data
     * @apiNote Zero position is implementation-defined.
     */
    default Angle getPosition() {
        throw new UnsupportedOperationException();
    }

    /**
     * Control the motor to turn to a specific position.
     *
     * @param position the target position
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void setPosition(Angle position) {
        throw new UnsupportedOperationException("the motor does not support turning by position");
    }

    /**
     * Get current angular velocity of the motor.
     *
     * @return the angular velocity
     * @throws UnsupportedOperationException if the motor does not support retrieving such data
     */
    default AngularVelocity getVelocity() {
        throw new UnsupportedOperationException();
    }

    /**
     * Control the motor by its output angular velocity.
     *
     * @param angularVelocity the angular velocity
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void setVelocity(AngularVelocity angularVelocity) {
        throw new UnsupportedOperationException("the motor does not support output by angular velocity");
    }

    /**
     * Get current output voltage of the motor.
     *
     * @return the voltage
     * @throws UnsupportedOperationException if the motor does not support retrieving such data
     */
    default Voltage getVoltage() {
        throw new UnsupportedOperationException();
    }

    /**
     * Control the motor by its output voltage.
     *
     * @param voltage the voltage
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void setVoltage(Voltage voltage) {
        throw new UnsupportedOperationException("the motor does not support output by voltage");
    }

    /**
     * Get current electric current of the motor.
     *
     * @return the electric current
     * @throws UnsupportedOperationException if the motor does not support retrieving such data
     */
    default Current getCurrent() {
        throw new UnsupportedOperationException();
    }

    /**
     * Control the motor by its output current.
     *
     * @param current the current
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void setCurrent(Current current) {
        throw new UnsupportedOperationException("the motor does not support output by current");
    }
}
