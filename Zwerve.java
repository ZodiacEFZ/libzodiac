package frc.libzodiac;

import frc.libzodiac.ui.Axis;
import frc.libzodiac.ui.Button;
import frc.libzodiac.util.Vec2D;

/**
 * A highly implemented class for hopefully all types of swerve control.
 */
public abstract class Zwerve extends Zubsystem implements ZmartDash {
    private static final double ROTATION_KP = 0.2;
    private static final double ROTATION_FAST = 1.5;
    private static final double ROTATION_NORMAL = 0.75;
    private static final double ROTATION_SLOW = 0.375;
    private static final double OUTPUT_FAST = 600;
    private static final double OUTPUT_NORMAL = 300;
    private static final double OUTPUT_SLOW = 100;
    public final Vec2D shape;
    /**
     * Gyro.
     */
    public final Axis yaw;
    /**
     * Swerve modules of a rectangular chassis.
     */
    public final Module[] module = new Module[4];
    public boolean headless = false;
    public double headless_zero = 0;
    /**
     * Modifier timed at the output speed of the chassis.
     */
    public double output = OUTPUT_NORMAL;
    public double rotation_output = ROTATION_NORMAL;
    private double desired_yaw;

    /**
     * Creates a new Zwerve.
     *
     * @param yaw_gyro The gyroscope axis measuring yaw.
     * @param shape    Shape of the robot, <code>x</code> for length and
     *                 <code>y</code> for width.
     */
    public Zwerve(Module front_left, Module front_right, Module rear_left, Module rear_right, Axis yaw_gyro, Vec2D shape) {
        this.module[SwerveModule.FRONT_LEFT] = front_left;
        this.module[SwerveModule.FRONT_RIGHT] = front_right;
        this.module[SwerveModule.REAR_LEFT] = rear_left;
        this.module[SwerveModule.REAR_RIGHT] = rear_right;
        this.yaw = yaw_gyro;
        this.shape = shape;
        this.desired_yaw = this.yaw.get();
        this.mod_reset();
    }

    /**
     * Method to calculate the radius of the rectangular robot.
     */
    private double radius() {
        return this.shape.div(2).r();
    }

    /**
     * Get the absolute current direction of the robot.j
     */
    public double curr_dir() {
        return this.yaw.get() - this.headless_zero;
    }

    /**
     * Get the direction adjustment applied under headless mode.
     */
    private double fix_dir() {
        if (this.yaw == null) {
            return 0;
        }
        return this.headless ? -this.curr_dir() : 0;
    }

    /**
     * Kinematics part rewritten using vector calculations.
     *
     * @param vel translational velocity, with +x as the head of the bot
     * @param rot rotate velocity, CCW positive
     */
    public Zwerve go(Vec2D vel, double rot) {
        final var curr_yaw = this.yaw.get();
        this.desired_yaw += rot * this.rotation_output;
        this.debug("desired", this.desired_yaw);

        final var rt = (this.desired_yaw - curr_yaw) * ROTATION_KP;

        final var vt = vel.rot(this.fix_dir());

        this.debug("translation", "" + vt);
        final var l = this.shape.x / 2;
        final var w = this.shape.y / 2;
        Vec2D[] v = new Vec2D[4];
        v[SwerveModule.FRONT_LEFT] = new Vec2D(l, w);
        v[SwerveModule.FRONT_RIGHT] = new Vec2D(l, -w);
        v[SwerveModule.REAR_LEFT] = new Vec2D(-l, w);
        v[SwerveModule.REAR_RIGHT] = new Vec2D(-l, -w);
        for (var i = 0; i < 4; i++) {
            v[i] = v[i].rot(Math.PI / 2).with_r(rt).add(vt);
        }
        for (int i = 0; i < 4; i++) {
            this.module[i].go(v[i].mul(this.output));
        }
        return this;
    }

    /**
     * Kinematics part rewritten using vector calculations.
     *
     * @param vel translational velocity, with +x as the head of the bot
     * @param yaw target yaw
     */
    public Zwerve go_yaw(Vec2D vel, double yaw) {
        this.desired_yaw = yaw;
        this.go(vel, 0);
        return this;
    }

    @Override
    public Zwerve update() {
        this.debug("headless", this.headless);
        this.debug("yaw", this.yaw.get());
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
        this.headless_zero = this.yaw.get();
    }

    public ZCommand drive(Axis x, Axis y, Axis rot, Button fast, Button slow) {
        return new Zambda(this, () -> {
            if (fast.down() && !slow.down()) {
                output = OUTPUT_FAST;
                rotation_output = ROTATION_FAST;
            } else if (!fast.down() && slow.down()) {
                output = OUTPUT_SLOW;
                rotation_output = ROTATION_SLOW;
            } else {
                output = OUTPUT_NORMAL;
                rotation_output = ROTATION_NORMAL;
            }
            final var vel = new Vec2D(x.get(), y.get());
            this.go(vel, rot.get());
        });
    }

    @Override
    public String key() {
        return "Zwerve";
    }

    public Zwerve mod_reset() {
        for (final var i : this.module) {
            i.reset();
        }
        return this;
    }

    /**
     * Defines one swerve module.
     */
    public interface Module {
        Module go(Vec2D velocity);

        Module reset();

        Module shutdown();
    }

    private static class SwerveModule {
        private static final int FRONT_LEFT = 0;
        private static final int FRONT_RIGHT = 3;
        private static final int REAR_LEFT = 1;
        private static final int REAR_RIGHT = 2;
    }
}
