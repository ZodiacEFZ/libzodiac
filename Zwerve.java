package frc.libzodiac;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.libzodiac.hardware.Pigeon;
import frc.libzodiac.ui.Axis;
import frc.libzodiac.ui.Button;
import frc.libzodiac.util.Vec2D;

/**
 * A highly implemented class for hopefully all types of swerve control.
 */
public abstract class Zwerve extends Zubsystem implements ZmartDash {
    private static final double ROTATION_KP = 0.2;
    private static final double OUTPUT_FAST = 6;
    private static final double OUTPUT_NORMAL = 3;
    private static final double OUTPUT_SLOW = 1;
    public static SwerveDriveKinematics kinematics;
    private final Pigeon gyro;
    private final ZInertialNavigation inav;
    private final Module front_left;
    private final Module rear_left;
    private final Module front_right;
    private final Module rear_right;
    private final SwerveDriveOdometry odometry;
    private boolean headless = false;
    private double headless_zero = 0;

    public Zwerve(Module front_left, Module front_right, Module rear_left, Module rear_right, Pigeon gyro, double length, double width) {
        this.front_left = front_left.reset();
        this.front_right = front_right.reset();
        this.rear_left = rear_left.reset();
        this.rear_right = rear_right.reset();
        this.gyro = gyro;
        this.inav = new ZInertialNavigation(gyro);
        kinematics = new SwerveDriveKinematics(new Translation2d(length / 2, width / 2), new Translation2d(length / 2, -width / 2), new Translation2d(-length / 2, width / 2), new Translation2d(-length / 2, -width / 2));
        this.odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), new SwerveModulePosition[]{
                front_left.getPosition(), front_right.getPosition(), rear_left.getPosition(), rear_right.getPosition()
        });
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(this.gyro.get() - headless_zero);
    }

    @Override
    public Zwerve update() {
        odometry.update(getRotation2d(), new SwerveModulePosition[]{
                front_left.getPosition(), front_right.getPosition(), rear_left.getPosition(), rear_right.getPosition()
        });
        inav.update();
        return this;
    }

    /**
     * Enable/disable headless mode.
     *
     * @param status whether to enable headless mode
     */
    public Zwerve headless(boolean status) {
        this.headless = status;
        return this;
    }

    /**
     * Enable headless mode.
     */
    public Zwerve headless() {
        return this.headless(true);
    }

    public Zwerve toggle_headless() {
        this.headless = !this.headless;
        return this;
    }

    public void reset_headless() {
        this.headless_zero = this.gyro.get();
    }

    public ZCommand drive(Axis x, Axis y, Axis rx, Axis ry, Button fast, Button slow, Button rotation) {
        return new Zambda(this, () -> {
            var rot = rotation.down() ? rx.get() : (new Vec2D(rx.get(), ry.get()).theta() - gyro.get()) * ROTATION_KP;
            var speed = headless ? ChassisSpeeds.fromFieldRelativeSpeeds(x.get(), y.get(), rot, getRotation2d()) : new ChassisSpeeds(x.get(), y.get(), rot);
            var states = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speed, 0.02));
            var output = OUTPUT_NORMAL;
            if (fast.down() && !slow.down()) {
                output = OUTPUT_FAST;
            } else if (!fast.down() && slow.down()) {
                output = OUTPUT_SLOW;
            }
            SwerveDriveKinematics.desaturateWheelSpeeds(states, output);
            front_left.go(states[0]);
            front_right.go(states[1]);
            rear_left.go(states[2]);
            rear_right.go(states[3]);
        });
    }

    @Override
    public String key() {
        return "Zwerve";
    }

    /**
     * Defines one swerve module.
     */
    public interface Module {
        Module go(SwerveModuleState desiredState);

        SwerveModulePosition getPosition();

        Module reset();

        Module shutdown();

        Module invert(boolean speed, boolean angle);

        Module set_pid(PIDController v, PIDController a);
    }
}
