package frc.libzodiac.hardware;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import frc.libzodiac.api.ZMotor;

/**
 * <i>Talon FX/i> motor, such as Falcon 500 and Kraken X60/X44.
 */
public final class TalonFXMotor implements ZMotor {
    public static final double TALONFX_UNIT = 2 * Math.PI;

    private final TalonFX motor;
    int output = 1;

    public TalonFXMotor(int can_id) {
        this.motor = new TalonFX(can_id);
    }

    public TalonFXMotor(int can_id, double kP, double kI, double kD) {
        this.motor = new TalonFX(can_id);
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        this.motor.getConfigurator().apply(new TalonFXConfiguration().withSlot0(slot0Configs));
    }

    public void factoryDefault(int can_id, TalonFXConfiguration config) {
        this.motor.getConfigurator().apply(new TalonFXConfiguration());
    }

    /**
     * Configure the PID controller arguments of this motor.
     *
     * @param kP proportional control coefficient
     * @param kI integral control coefficient
     * @param kD derivative control coefficient
     */
    public void setPid(double kP, double kI, double kD) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        this.motor.getConfigurator().apply(slot0Configs);
    }

    public void setInverted(boolean inverted) {
        output = inverted ? -1 : 1;
    }

    public void invert() {
        output = -output;
    }

    @Override
    public void shutdown() {
        this.motor.setControl(new CoastOut());
    }

    @Override
    public void brake() {
        this.motor.setControl(new StaticBrake());
    }

    @Override
    public void power(double ratio) {
        this.motor.set(output * ratio);
    }

    @Override
    public void angle(double rad) {
        this.motor.setControl(new PositionDutyCycle(output * rad / TALONFX_UNIT));
    }

    @Override
    public void velocity(double rads) {
        // Our practice suggest that `VelocityVoltage` api produces a somehow more
        // stable output than `VelocityDutyCycle`.
        this.motor.setControl(new VelocityVoltage(output * rads / TALONFX_UNIT));
    }

    @Override
    public void voltage(double volt) {
        this.motor.setControl(new VoltageOut(output * volt));
    }

    public double getPosition() {
        return output * this.motor.getPosition().getValue().in(Units.Radians);
    }

    public void setPosition(double rad) {
        this.motor.setPosition(output * rad / TALONFX_UNIT);
    }

    public double getVelocity() {
        return output * this.motor.getVelocity().getValue().in(Units.RadiansPerSecond);
    }

    /**
     * Attain an access point to the internal motor
     *
     * @return the motor
     */
    public TalonFX motor() {
        return this.motor;
    }
}
