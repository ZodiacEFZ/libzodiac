package frc.libzodiac;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import frc.libzodiac.hardware.Pigeon;
import frc.libzodiac.ui.Axis;
import frc.libzodiac.ui.Axis2D;
import frc.libzodiac.ui.Button;
import frc.libzodiac.util.Vec2D;

/**
 * A highly implemented class for hopefully all types of swerve control.
 */
public abstract class Zwerve extends Zubsystem {
    private static final double ROTATION_KP = 0.05;
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
    private final double field_zero;
    private boolean headless = false;
    private double headless_zero;

    public Zwerve(Module front_left, Module front_right, Module rear_left, Module rear_right, Pigeon gyro, Vec2D size, Pose2d initialPose) {
        this.front_left = front_left.reset();
        this.front_right = front_right.reset();
        this.rear_left = rear_left.reset();
        this.rear_right = rear_right.reset();
        this.gyro = gyro;
        this.inav = new ZInertialNavigation(gyro).set_zero();
        this.field_zero = this.headless_zero = this.gyro.get();
        final var width = size.x;
        final var length = size.y;
        kinematics = new SwerveDriveKinematics(new Translation2d(length / 2, width / 2), new Translation2d(length / 2, -width / 2), new Translation2d(-length / 2, width / 2), new Translation2d(-length / 2, -width / 2));
        this.odometry = new SwerveDriveOdometry(kinematics, getFieldRotation(), new SwerveModulePosition[]{
                front_left.getPosition(), front_right.getPosition(), rear_left.getPosition(), rear_right.getPosition()
        }, initialPose);
    }

    public Rotation2d getHeadlessRotation() {
        return Rotation2d.fromRadians(this.gyro.get() - headless_zero);
    }

    public Rotation2d getFieldRotation() {
        return Rotation2d.fromRadians(this.gyro.get() - field_zero);
    }

    public Zwerve update() {
        odometry.update(getFieldRotation(), new SwerveModulePosition[]{
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

    public boolean toggle_headless() {
        this.headless = !this.headless;
        return headless;
    }

    public void reset_headless() {
        this.headless_zero = this.gyro.get();
    }

    public ZCommand drive(Axis2D l, Axis2D r, Axis lx, Axis ly, Axis rx, Axis ry, Button fast, Button slow, Button rotation) {
        return new Zambda(this, () -> {
            var output = OUTPUT_NORMAL;
            if (fast.down() && !slow.down()) {
                output = OUTPUT_FAST;
            } else if (!fast.down() && slow.down()) {
                output = OUTPUT_SLOW;
            }

            final var lv = l.vec().mul(output);
            final var rv = r.vec();
            final var target_theta = rv.r() > 0.7 ? rv.theta() : this.gyro.get();
            final var delta = new Rotation2d(target_theta - this.gyro.get()).getRadians();
            var rot = rotation.down() ? rx.get() : delta * ROTATION_KP;

            var speed = headless ? ChassisSpeeds.fromFieldRelativeSpeeds(lv.x, lv.y, rot, getHeadlessRotation()) : new ChassisSpeeds(lv.x, lv.y, rot);

            var states = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speed, 0.02));
            SwerveDriveKinematics.desaturateWheelSpeeds(states, output);
            front_left.go(states[0]);
            front_right.go(states[1]);
            rear_left.go(states[2]);
            rear_right.go(states[3]);
        });
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
