package frc.libzodiac.hardware;

import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.libzodiac.api.ZMotor;
import frc.libzodiac.util.Lazy;

/**
 * The <i>Falcon 500</i> motor.
 */
public final class Falcon implements ZMotor {

    /**
     * Falcon uses rotations as its position unit.
     */
    public static double POSITION_UNIT = 2 * Math.PI;

    /**
     * Falcon uses rotations per second as its velocity unit.
     */
    public static double VELOCITY_UNIT = 2 * Math.PI;

    private final Lazy<TalonFX> motor;

    private Optional<Slot0Configs> pid = Optional.empty();

    private boolean inverted = false;

    private Falcon(int can_id, Optional<Slot0Configs> pid, boolean inverted) {
        this.motor = Lazy.with(() -> new TalonFX(can_id)).then(motor -> {
            if (this.pid.isPresent())
                motor.getConfigurator().apply(this.pid.get());
            motor.setInverted(this.inverted);
        });
        this.pid = pid;
        this.inverted = inverted;
    }

    /**
     * Bind to a <i>Falcon 700</i> motor.
     *
     * @param can_id id on the CAN bus
     */
    public static Falcon with(int can_id) {
        return new Falcon(can_id, Optional.empty(), false);
    }

    /**
     * Configure the PID controller arguments of this motor.
     * 
     * @param k_p proportional control coefficient
     * @param k_i integral control coefficient
     * @param k_d derivative control coefficient
     * @return self for chaining
     */
    public Falcon config_pid(double k_p, double k_i, double k_d) {
        this.pid = Optional.of(new Slot0Configs().withKP(k_p).withKI(k_i).withKD(k_d));
        return this;
    }

    /**
     * Configure the PID controller profile.
     * 
     * @param cfg the PID profile to apply
     * @return self for chaining
     */
    public Falcon config_pid(Slot0Configs cfg) {
        this.pid = Optional.of(cfg);
        return this;
    }

    /**
     * Configure the inversion state of this motor.
     * 
     * @param inverted whether to invert
     * @return self for chaining
     * 
     * @implNote This API directly calls the internal <code>TalonFX</code>'s
     *           inversion function during setup, thus invocation after the lazy
     *           initialization would take no effect.
     */
    public Falcon config_inversion(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    /**
     * Invert the motor. Short for <code>.config_inversion</code>.
     * 
     * @return self for chaining
     */
    public Falcon invert() {
        return this.config_inversion(!this.inverted);
    }

    @Override
    public void shutdown() {
        this.motor.get().setControl(new CoastOut());
    }

    @Override
    public void brake() {
        this.motor.get().setControl(new StaticBrake());
    }

    @Override
    public void power(double ratio) {
        this.motor.get().set(ratio);
    }

    @Override
    public void angle(double rad) {
        this.motor.get().setControl(new PositionDutyCycle(rad / POSITION_UNIT));
    }

    @Override
    public void velocity(double rads) {
        // Our practice suggest that `VelocityVoltage` api produces a somehow more
        // stable output than `VelocityDutyCycle`.
        this.motor.get().setControl(new VelocityVoltage(rads / VELOCITY_UNIT));
    }

    @Override
    public void voltage(double volt) {
        this.motor.get().setControl(new VoltageOut(volt));
    }

    /**
     * Attain an access point to the internal motor api.
     * 
     * @return the api
     */
    public TalonFX raw_api() {
        return this.motor.get();
    }

}
