package frc.libzodiac;

import edu.wpi.first.math.controller.PIDController;

import java.util.HashMap;

/**
 * Defines a large collection of APIs to operate various motors so that motors
 * can be controlled under a unified generic way.
 */
public abstract class ZMotor {
    /**
     * Motions profiles pre-defined for future use.
     */
    public final HashMap<String, Double> profile = new HashMap<>();
    /**
     * The PID configuration of this motor.
     */
    public boolean inverted = false;

    /**
     * Override this method to set the motor's PID to <code>this.pid</code>.
     */
    protected abstract ZMotor apply_pid(PIDController pid);

    /**
     * Set PID parameters.
     */
    public ZMotor set_pid(PIDController pid) {
        this.apply_pid(pid);
        return this;
    }

    /**
     * Set PID parameters.
     */
    public ZMotor set_pid(double k_p, double k_i, double k_d) {
        return this.set_pid(new PIDController(k_p, k_i, k_d));
    }

    /**
     * Stop any output behaviour of this motor.
     */
    public abstract ZMotor shutdown();

    /**
     * Stop the motor.
     * Using brake mode if it is available.
     */
    public abstract ZMotor stop(boolean stop);

    /**
     * Perform actions with the specified motion profile.
     */
    public abstract ZMotor go(String profile);

    /**
     * Set the output.
     *
     * @param rads rad/s for general motors and rad for servos.
     */
    public abstract ZMotor go(double rads);

    public abstract ZMotor raw(double output);
}