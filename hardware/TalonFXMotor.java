package frc.libzodiac.hardware;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.controller.PIDController;
import frc.libzodiac.ZMotor;
import frc.libzodiac.Zervo;

public class TalonFXMotor extends ZMotor {
    public static final double VELOCITY_RAW_UNIT = 2 * Math.PI;

    protected final TalonFX motor;

    public TalonFXMotor(int can_id) {
        this.motor = new TalonFX(can_id);
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
    public static Slot0Configs PIDConfig(double kP, double kI, double kD, GravityTypeValue gravityType, double kG, double kS, double kV, double kA) {
        return new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withGravityType(gravityType).withKG(kG).withKS(kS).withKV(kV).withKA(kA);
    }

    public static Slot0Configs PIDConfig(double kP, double kI, double kD, double kS) {
        return PIDConfig(kP, kI, kD, GravityTypeValue.Elevator_Static, 0, kS, 0, 0);
    }

    public static MotionMagicConfigs MotionMagicConfig(double v, double a, double j) {
        return MotionMagicConfig(0, 0, v, a, j);
    }

    public static MotionMagicConfigs MotionMagicConfig(double a, double j) {
        return MotionMagicConfig(0, 0, 0, a, j);
    }

    public static MotionMagicConfigs MotionMagicConfig(double kV, double kA, double v, double a, double j) {
        return new MotionMagicConfigs().withMotionMagicExpo_kV(kV).withMotionMagicExpo_kA(kA).withMotionMagicCruiseVelocity(v).withMotionMagicAcceleration(a).withMotionMagicJerk(j);
    }

    public static TalonFXConfiguration PIDMotionMagicConfig(Slot0Configs pid, MotionMagicConfigs motionMagic) {
        return new TalonFXConfiguration().withSlot0(pid).withMotionMagic(motionMagic);
    }

    @Override
    protected TalonFXMotor apply_pid(PIDController pid) {
        this.motor.getConfigurator().apply(new Slot0Configs().withKP(pid.getP()).withKI(pid.getI()).withKD(pid.getD()));
        return this;
    }

    public ZMotor set_pid(Slot0Configs config) {
        this.motor.getConfigurator().apply(config);
        return this;
    }

    public ZMotor set_motion_magic(TalonFXConfiguration config) {
        this.motor.getConfigurator().apply(config);
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
        return this.go(inverted ? -v : v);
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
        this.motor.set(inverted ? -output : output);
        return this;
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

    public static class Servo extends TalonFXMotor implements Zervo {
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
            final var pos = this.inverted ? -rad : rad;
            final var v = new PositionDutyCycle(pos / Servo.POSITION_RAW_UNIT);
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
    }

    public static final class MotionMagicServo extends Servo {
        public MotionMagicServo(int can_id) {
            super(can_id);
        }

        @Override
        public MotionMagicServo go(double rad) {
            final var pos = this.inverted ? -rad : rad;
            final var v = new MotionMagicVoltage(pos / Servo.POSITION_RAW_UNIT);
            this.motor.setControl(v);
            return this;
        }

        @Override
        public MotionMagicServo go_v(double rad_s) {
            final var vel = this.inverted ? -rad_s : rad_s;
            final var v = new MotionMagicVelocityVoltage(vel / TalonFXMotor.VELOCITY_RAW_UNIT);
            this.motor.setControl(v);
            return this;
        }

        public MotionMagicServo go_expo(double rad) {
            final var pos = this.inverted ? -rad : rad;
            final var v = new MotionMagicExpoVoltage(pos / Servo.POSITION_RAW_UNIT);
            this.motor.setControl(v);
            return this;
        }
    }
}
