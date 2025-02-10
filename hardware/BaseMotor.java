package frc.libzodiac.hardware;

/**
 * Defines a large collection of APIs to operate various motors so that motors
 * can be controlled under a unified generic way.
 */
public interface BaseMotor {
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
     * @param ratio the percent power in [-1,1]
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void power(double ratio) {
        throw new UnsupportedOperationException("the motor does not support output by percent power");
    }

    /**
     * Control the motor to turn to a specific angle.
     *
     * @param rad the angle in radian
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     * @apiNote Zero position of the angle is implementation defined.
     */
    default void angle(double rad) {
        throw new UnsupportedOperationException("the motor does not support turning by angle");
    }

    /**
     * Control the motor by its output angular velocity.
     *
     * @param rads the velocity in rad/s
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void velocity(double rads) {
        throw new UnsupportedOperationException("the motor does not support output by velocity");
    }

    /**
     * Control the motor by its output voltage.
     *
     * @param volt the voltage in V
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void voltage(double volt) {
        throw new UnsupportedOperationException("the motor does not support output by voltage");
    }

    /**
     * Control the motor by its output current.
     *
     * @param amp the current in A
     * @throws UnsupportedOperationException if the motor does not support such type
     *                                       of control
     */
    default void current(double amp) {
        throw new UnsupportedOperationException("the motor does not support output by current");
    }
}
