package frc.libzodiac.hardware.group;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.Zwerve.Module;
import frc.libzodiac.hardware.MagEncoder;
import frc.libzodiac.hardware.TalonFXMotor;

public final class TalonFXSwerve implements Module, ZmartDash {
    private static final double SWERVE_RATIO = 150.0 / 7.0;
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
        final var target = curr * SWERVE_RATIO;
        this.angle_motor.set_zero(target);
        return this;
    }

    @Override
    public TalonFXSwerve go(SwerveModuleState desiredState) {
        final var encoderRotation = new Rotation2d(this.angle_motor.get());

        final var state = SwerveModuleState.optimize(desiredState, encoderRotation);

        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

        this.speed_motor.go_v(state.speedMetersPerSecond);
        this.angle_motor.go(state.angle.getRadians() * SWERVE_RATIO);
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
        a.setIntegratorRange(-Math.PI, Math.PI);
        this.speed_motor.set_pid(v);
        this.angle_motor.set_pid(a);
        return this;
    }

    @Override
    public String key() {
        return "TalonFXSwerve(" + this.speed_motor.key() + ",...)";
    }
}
