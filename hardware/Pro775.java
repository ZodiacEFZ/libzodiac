package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import frc.libzodiac.ZMotor;
import frc.libzodiac.util.Lazy;

import java.util.Optional;

/**
 * The <i>Vex 775 pro</i> motor.
 */
public final class Pro775 implements ZMotor {

    private final Lazy<TalonSRX> motor;
    /**
     * Unit of the sensor wired to the <i>TalonSRX</i> encoder, represents how many
     * radians a raw sensor unit is equal to.
     */
    public double unit = 1;
    public Profile profile = new Profile();
    private Optional<PIDController> pid = Optional.empty();

    private Pro775(int can_id) {
        this.motor = new Lazy<>(() -> new TalonSRX(can_id)).then(motor -> {
            if (this.pid.isPresent()) {
                motor.config_kP(0, this.pid.get().getP());
                motor.config_kI(0, this.pid.get().getI());
                motor.config_kD(0, this.pid.get().getD());
            }
        });
    }

    /**
     * Bind to a <i>TalonSRX</i> encoder.
     *
     * @param can_id id on the CAN bus
     */
    public static Pro775 with(int can_id) {
        return new Pro775(can_id);
    }

    /**
     * Configure PID arguments of the motor.
     *
     * @param k_p PID proportional term factor
     * @param k_i PID integral term factor
     * @param k_d PID derivative term factor
     * @return self for chaining
     */
    public Pro775 config_pid(double k_p, double k_i, double k_d) {
        this.pid = Optional.of(new PIDController(k_p, k_i, k_d));
        return this;
    }

    /**
     * Configure the sensor unit on the encoder to make methods like
     * <code>.angle(double)</code> work properly under radians.
     *
     * @param unit how many radians a sensor raw unit is equal to
     * @return self for chaining
     */
    public Pro775 config_unit(double unit) {
        this.unit = unit;
        return this;
    }

    @Override
    public void shutdown() {
        this.motor.get().setNeutralMode(NeutralMode.Coast);
        this.motor.get().neutralOutput();
    }

    @Override
    public void brake() {
        this.motor.get().setNeutralMode(NeutralMode.Brake);
        this.motor.get().neutralOutput();
    }

    @Override
    public void power(double ratio) {
        this.motor.get().set(ControlMode.PercentOutput, ratio);
    }

    /**
     * @implNote Unit of the angle may not necessarily be radian but dependant on
     *           sensor
     *           wired to the <i>TalonSRX</i> encoder unless <code>.unit</code> is
     *           properly set.
     */
    @Override
    public void angle(double rad) {
        this.motor.get().set(ControlMode.Position, rad);
    }

    @Override
    public void velocity(double rads) {
        this.motor.get().set(ControlMode.Velocity, rads);
    }

    @Override
    public void current(double amp) {
        this.motor.get().set(ControlMode.Current, amp);
    }

    public Pro775 reset() {
        this.motor.get().setSelectedSensorPosition(0);
        return this;
    }

    public double get() {
        return this.motor.get().getSelectedSensorPosition();
    }
}
