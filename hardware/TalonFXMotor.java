package frc.libzodiac.hardware;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.libzodiac.ZMotor;
import frc.libzodiac.Zervo;
import frc.libzodiac.ZmartDash;

public class TalonFXMotor extends ZMotor implements ZmartDash {
    public static final double VELOCITY_RAW_UNIT = 2 * Math.PI;

    protected TalonFX motor;

    public TalonFXMotor(int can_id) {
        this.motor = new TalonFX(can_id);
    }

    @Override
    protected TalonFXMotor apply_pid() {
        this.motor.getConfigurator().apply(new Slot0Configs().withKP(this.pid.k_p).withKI(this.pid.k_i).withKD(this.pid.k_d));
        return this;
    }

    @Override
    public TalonFXMotor shutdown() {
        this.motor.stopMotor();
        return this;
    }

    @Override
    public TalonFXMotor stop(boolean stop) {
        if (stop) {
            this.motor.setControl(new StaticBrake());
        } else {
            this.motor.setControl(new CoastOut());
        }
        return this;
    }

    @Override
    public TalonFXMotor go(String profile) {
        final var v = this.profile.get(profile);
        return this.go(v);
    }

    @Override
    public TalonFXMotor go(double rad_s) {
        final var vel = this.inverted ? -rad_s : rad_s;
        final var v = new VelocityDutyCycle(vel / TalonFXMotor.VELOCITY_RAW_UNIT);
        this.motor.setControl(v);
        return this;
    }

    @Override
    public TalonFXMotor raw(double output) {
        this.motor.set(output);
        return this;
    }

    @Override
    public String key() {
        return this.motor.getDescription();
    }

    public TalonFXMotor invert(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public TalonFXMotor invert() {
        return this.invert(true);
    }

    public TalonFXMotor go_v(double rad_s) {
        final var vel = this.inverted ? -rad_s : rad_s;
        final var v = new VelocityVoltage(vel / TalonFXMotor.VELOCITY_RAW_UNIT);
        this.motor.setControl(v);
        return this;
    }

    public static final class Servo extends TalonFXMotor implements Zervo {
        public static final double POSITION_RAW_UNIT = 2 * Math.PI;

        public Servo(int can_id) {
            super(can_id);
        }

        @Override
        public Servo go(String profile) {
            final var v = this.profile.get(profile);
            return this.go(v);
        }

        @Override
        public Servo go(double rad) {
            final var v = new PositionDutyCycle((this.inverted ? -rad : rad) / Servo.POSITION_RAW_UNIT);
            this.motor.setControl(v);
            return this;
        }

        @Override
        public Servo set_zero(double zero) {
            final var v = zero / POSITION_RAW_UNIT;
            this.motor.setPosition(this.inverted ? -v : v);
            return this;
        }

        @Override
        public double get() {
            final var v = this.motor.getPosition().refresh().getValue() * POSITION_RAW_UNIT;
            return this.inverted ? -v : v;
        }

        @Override
        public Servo invert(boolean inverted) {
            this.inverted = inverted;
            return this;
        }

        @Override
        public Servo invert() {
            return this.invert(true);
        }

        @Override
        public String key() {
            return "Servo " + this.motor.getDescription();
        }
    }
}
