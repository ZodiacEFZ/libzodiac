package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.libzodiac.ZMotor;
import frc.libzodiac.Zervo;

public class Pro775 extends ZMotor {
    protected final TalonSRX motor;

    public Pro775(int can_id) {
        this.motor = new TalonSRX(can_id);
    }

    @Override
    protected Pro775 apply_pid() {
        this.motor.config_kP(0, this.pid.getP());
        this.motor.config_kI(0, this.pid.getI());
        this.motor.config_kD(0, this.pid.getD());
        return this;
    }

    @Override
    public Pro775 shutdown() {
        this.motor.setNeutralMode(NeutralMode.Coast);
        this.motor.neutralOutput();
        return this;
    }

    @Override
    public Pro775 stop(boolean stop) {
        if (stop) {
            this.motor.setNeutralMode(NeutralMode.Brake);
            this.motor.neutralOutput();
        } else {
            this.motor.setNeutralMode(NeutralMode.Coast);
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
        this.motor.set(ControlMode.Velocity, inverted ? -raw_unit : raw_unit);
        return this;
    }

    @Override
    public Pro775 raw(double output) {
        this.motor.set(ControlMode.PercentOutput, inverted ? -output : output);
        return this;
    }

    public static class Servo extends Pro775 implements Zervo {
        public Servo(int can_id) {
            super(can_id);
        }

        @Override
        public Zervo set_zero(double zero) {
            this.motor.setSelectedSensorPosition(zero);
            return this;
        }

        @Override
        public double get() {
            final var v = this.motor.getSelectedSensorPosition();
            return inverted ? -v : v;
        }

        @Override
        public Servo go(String profile) {
            final var v = this.profile.get(profile);
            this.motor.set(ControlMode.Position, inverted ? -v : v);
            return this;
        }

        @Override
        public Servo go(double raw_unit) {
            this.motor.set(ControlMode.Position, inverted ? -raw_unit : raw_unit);
            return this;
        }
    }
}
