package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import frc.libzodiac.ZMotor;
import frc.libzodiac.Zervo;
import frc.libzodiac.util.Lazy;

public class Pro775 extends ZMotor {
    protected final Lazy<TalonSRX> motor;

    public Pro775(int can_id) {
        this.motor = new Lazy<>(() -> new TalonSRX(can_id));
    }

    @Override
    protected Pro775 apply_pid(PIDController pid) {
        this.motor.get().config_kP(0, pid.getP());
        this.motor.get().config_kI(0, pid.getI());
        this.motor.get().config_kD(0, pid.getD());
        return this;
    }

    @Override
    public Pro775 shutdown() {
        this.motor.get().setNeutralMode(NeutralMode.Coast);
        this.motor.get().neutralOutput();
        return this;
    }

    @Override
    public Pro775 stop(boolean stop) {
        if (stop) {
            this.motor.get().setNeutralMode(NeutralMode.Brake);
            this.motor.get().neutralOutput();
        } else {
            this.motor.get().setNeutralMode(NeutralMode.Coast);
        }
        return this;
    }

    @Override
    public Pro775 go(String profile) {
        final var v = this.profile.get(profile);
        return this.go(inverted ? -v : v);
    }

    @Override
    public Pro775 go(double raw_unit) {
        this.motor.get().set(ControlMode.Velocity, inverted ? -raw_unit : raw_unit);
        return this;
    }

    @Override
    public Pro775 raw(double output) {
        this.motor.get().set(ControlMode.PercentOutput, inverted ? -output : output);
        return this;
    }

    public static class Servo extends Pro775 implements Zervo {
        public Servo(int can_id) {
            super(can_id);
        }

        @Override
        public Zervo set_zero(double zero) {
            this.motor.get().setSelectedSensorPosition(zero);
            return this;
        }

        @Override
        public double get() {
            final var v = this.motor.get().getSelectedSensorPosition();
            return inverted ? -v : v;
        }

        @Override
        public Servo go(String profile) {
            final var v = this.profile.get(profile);
            this.motor.get().set(ControlMode.Position, inverted ? -v : v);
            return this;
        }

        @Override
        public Servo go(double raw_unit) {
            this.motor.get().set(ControlMode.Position, inverted ? -raw_unit : raw_unit);
            return this;
        }
    }
}
