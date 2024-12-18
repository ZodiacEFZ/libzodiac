package frc.libzodiac.hardware;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.controller.PIDController;
import frc.libzodiac.api.ZMotor;
import frc.libzodiac.util.Lazy;

public class TalonFXMotor implements ZMotor {
    public static final double VELOCITY_RAW_UNIT = 2 * Math.PI;

    protected final Lazy<TalonFX> motor;
    boolean inverted = false;

    public TalonFXMotor(int can_id) {
        this.motor = new Lazy<>(() -> new TalonFX(can_id));
    }

    public static Slot0Configs PIDConfig(double kP, double kI, double kD, double kS) {
        return PIDConfig(kP, kI, kD, GravityTypeValue.Elevator_Static, 0, kS, 0, 0);
    }

    /**
     * @param kP          Proportional Gain
     * @param kI          Integral Gain
     * @param kD          Derivative Gain
     * @param gravityType Gravity Feedforward/Feedback Type
     * @param kG          Gravity Feedforward/Feedback Gain
     * @param kS          Static Feedforward Gain
     * @param kV          Velocity Feedforward Gain
     * @param kA          Acceleration Feedforward Gain
     */
    public static Slot0Configs PIDConfig(double kP, double kI, double kD, GravityTypeValue gravityType, double kG,
            double kS, double kV, double kA) {
        return new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withGravityType(gravityType).withKG(kG).withKS(kS)
                .withKV(kV).withKA(kA);
    }

    public static MotionMagicConfigs MotionMagicConfig(double v, double a, double j) {
        return MotionMagicConfig(0, 0, v, a, j);
    }

    public static MotionMagicConfigs MotionMagicConfig(double kV, double kA, double v, double a, double j) {
        return new MotionMagicConfigs().withMotionMagicExpo_kV(kV).withMotionMagicExpo_kA(kA)
                .withMotionMagicCruiseVelocity(v).withMotionMagicAcceleration(a).withMotionMagicJerk(j);
    }

    public static MotionMagicConfigs MotionMagicConfig(double a, double j) {
        return MotionMagicConfig(0, 0, 0, a, j);
    }

    public static TalonFXConfiguration PIDMotionMagicConfig(Slot0Configs pid, MotionMagicConfigs motionMagic) {
        return new TalonFXConfiguration().withSlot0(pid).withMotionMagic(motionMagic);
    }

    public TalonFXMotor set_pid(PIDController pid) {
        this.motor.get().getConfigurator()
                .apply(new Slot0Configs().withKP(pid.getP()).withKI(pid.getI()).withKD(pid.getD()));
        return this;
    }

    public TalonFXMotor set_pid(Slot0Configs config) {
        this.motor.get().getConfigurator().apply(config);
        return this;
    }

    public TalonFXMotor set_motion_magic(TalonFXConfiguration config) {
        this.motor.get().getConfigurator().apply(config);
        return this;
    }

    @Override
    public void shutdown() {
        this.motor.get().setControl(new CoastOut());
        this.motor.get().stopMotor();
    }

    @Override
    public void brake() {
        this.motor.get().setControl(new StaticBrake());
        this.motor.get().stopMotor();
    }

    @Override
    public void power(double output) {
        this.motor.get().set(inverted ? -output : output);
    }

    @Override
    public void velocity(double rad_s) {
        final var vel = this.inverted ? -rad_s : rad_s;
        final var v = new VelocityDutyCycle(vel / TalonFXMotor.VELOCITY_RAW_UNIT);
        this.motor.get().setControl(v);
    }

    public TalonFXMotor invert() {
        return this.invert(true);
    }

    public TalonFXMotor invert(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public TalonFXMotor velocity_v(double rad_s) {
        final var vel = this.inverted ? -rad_s : rad_s;
        final var v = new VelocityVoltage(vel / TalonFXMotor.VELOCITY_RAW_UNIT);
        this.motor.get().setControl(v);
        return this;
    }

    public static final double POSITION_RAW_UNIT = 2 * Math.PI;

    public double get() {
        final var v = this.motor.get().getPosition().refresh().getValue() * POSITION_RAW_UNIT;
        return this.inverted ? -v : v;
    }

    public void set_zero(double zero) {
        final var v = zero / POSITION_RAW_UNIT;
        this.motor.get().setPosition(this.inverted ? -v : v);
    }

}
