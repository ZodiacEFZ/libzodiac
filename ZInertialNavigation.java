package frc.libzodiac;

import edu.wpi.first.wpilibj.Timer;
import frc.libzodiac.util.Vec2;

public class ZInertialNavigation implements ZDashboard.Dashboard {
    private final Gyro gyro;

    private final Timer timer = new Timer();
    private boolean started = false;
    private double zero = 0;
    private Vec2 pos = new Vec2(0, 0);
    private Vec2 speed = new Vec2(0, 0);

    public ZInertialNavigation(Gyro gyro) {
        this.gyro = gyro;
    }

    public ZInertialNavigation set_zero() {
        this.zero = this.gyro.getYaw();
        return this;
    }

    public ZInertialNavigation set_zero(double zero) {
        this.zero = zero;
        this.started = true;
        return this;
    }

    public ZInertialNavigation update() {
        if (!this.started) {
            return this;
        }
        final var loopTime = this.timer.get();
        this.timer.reset();

        final var acc = this.gyro.getAccelerationNoGravity().rot(-this.getYaw());
        this.speed = this.speed.add(acc.mul(loopTime));
        Vec2 dis = this.speed.mul(loopTime);
        this.pos = this.pos.add(dis);
        dashboardTab().add("posinav", "" + this.getPosition());
        dashboardTab().add("yawinav", this.getYaw());
        dashboardTab().add("acc", "" + acc);
        return this;
    }

    public Vec2 getPosition() {
        return pos;
    }

    public Vec2 getSpeed() {
        return this.speed;
    }

    public double getYaw() {
        return this.gyro.getYaw() - this.zero;
    }

    public interface Gyro {
        double getYaw();

        Vec2 getAccelerationNoGravity();
    }
}