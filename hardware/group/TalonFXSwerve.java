package frc.libzodiac.hardware.group;

import frc.libzodiac.Util;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.Zwerve.Module;
import frc.libzodiac.hardware.MagEncoder;
import frc.libzodiac.hardware.TalonFXMotor;
import frc.libzodiac.util.Vec2D;

public final class TalonFXSwerve implements Module, ZmartDash {
    private static final double SWERVE_RATIO = 150.0 / 7.0;
    public final TalonFXMotor speed_motor;
    public final TalonFXMotor.Servo angle_motor;
    private final MagEncoder encoder;

    public TalonFXSwerve(int speed_motor_id, int angle_motor_id, int encoder_id, double encoder_zero) {
        this.speed_motor = new TalonFXMotor(speed_motor_id);
        this.angle_motor = new TalonFXMotor.Servo(angle_motor_id);
        this.encoder = new MagEncoder(encoder_id).set_zero(encoder_zero);
    }

    @Override
    public TalonFXSwerve reset() {
        final var curr = this.encoder.get();
        final var target = curr * SWERVE_RATIO;
        this.angle_motor.set_zero(target);
        return this;
    }

    @Override
    public TalonFXSwerve go(Vec2D vel) {
        if (vel.r() == 0) {
            this.speed_motor.shutdown();
            this.angle_motor.shutdown();
            return this;
        }

        final var best = Util.swerve_optimize(this.angle_motor.get() / SWERVE_RATIO, vel.theta());

        this.speed_motor.go_v(best.x1 * vel.r());
        this.angle_motor.go(best.x0 * SWERVE_RATIO);
        return this;
    }

    @Override
    public TalonFXSwerve shutdown() {
        this.angle_motor.shutdown();
        this.speed_motor.shutdown();
        return this;
    }

    @Override
    public String key() {
        return "TalonFXSwerve(" + this.speed_motor.key() + ",...)";
    }
}
