package frc.libzodiac.hardware.group;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.libzodiac.Zwerve.Module;
import frc.libzodiac.hardware.MagEncoder;
import frc.libzodiac.hardware.TalonFXMotor;

public final class TalonFXSwerve implements Module {
    private static final double TURNING_RATIO = 150.0 / 7.0;
    private static final double WHEEL_CIRCUS = 0.1 * Math.PI;
    private static final double DRIVE_RATIO = 6.75;
    public final TalonFXMotor.Servo speed_motor;
    public final TalonFXMotor.Servo angle_motor;
    private final MagEncoder encoder;

    public TalonFXSwerve(int speed_motor_id, int angle_motor_id, int encoder_id, double encoder_zero) {
        this.speed_motor = new TalonFXMotor.Servo(speed_motor_id);
        this.angle_motor = new TalonFXMotor.Servo(angle_motor_id);
        this.encoder = new MagEncoder(encoder_id).set_zero(encoder_zero);
        this.reset();
    }

    @Override
    public TalonFXSwerve reset() {
        final var curr = this.encoder.get();
        final var target = curr * TURNING_RATIO;
        this.angle_motor.set_zero(target);
        return this;
    }

    @Override
    public TalonFXSwerve go(SwerveModuleState desiredState) {
        final var currentAngle = new Rotation2d(this.angle_motor.get() / TURNING_RATIO);

        final var state = SwerveModuleState.optimize(desiredState, currentAngle);
        state.speedMetersPerSecond *= Math.abs(state.angle.minus(currentAngle).getCos());
        var angle = this.angle_motor.get() / TURNING_RATIO + state.angle.minus(currentAngle).getRadians();

        this.speed_motor.go_v(state.speedMetersPerSecond * DRIVE_RATIO / WHEEL_CIRCUS * 2 * Math.PI);
        this.angle_motor.go(angle * TURNING_RATIO);
        return this;
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(speed_motor.get(), new Rotation2d(angle_motor.get()));
    }

    @Override
    public TalonFXSwerve shutdown() {
        this.angle_motor.shutdown();
        this.speed_motor.shutdown();
        return this;
    }

    @Override
    public TalonFXSwerve invert(boolean speed, boolean angle) {
        this.speed_motor.invert(speed);
        this.angle_motor.invert(angle);
        return this;
    }

    @Override
    public Module set_pid(PIDController v, PIDController a) {
        this.speed_motor.set_pid(v);
        this.angle_motor.set_pid(a);
        return this;
    }

    public Module set_pid(Slot0Configs v, Slot0Configs a) {
        this.speed_motor.set_pid(v);
        this.angle_motor.set_pid(a);
        return this;
    }
}
