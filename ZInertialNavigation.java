package frc.libzodiac;

import edu.wpi.first.wpilibj.Timer;
import frc.libzodiac.util.Vec2D;

public class ZInertialNavigation implements ZmartDash {
    private final Gyro gyro;

    private final Timer timer = new Timer();
    private final boolean started = false;
    private double zero = 0;
    private Vec2D pos = new Vec2D(0, 0);
    private Vec2D speed = new Vec2D(0, 0);

    public ZInertialNavigation(Gyro gyro) {
        this.gyro = gyro;
    }

    public ZInertialNavigation set_zero() {
        this.zero = this.gyro.getYaw();
        return this;
    }

    public ZInertialNavigation set_zero(double zero) {
        this.zero = zero;
        return this;
    }

    public ZInertialNavigation update() {
        if (!started) {
            this.set_zero();
            return this;
        }
        this.debug("posinav", "" + this.getPosition());
        this.debug("yawinav", this.getYaw());
        this.debug("acc", "" + this.gyro.getAccelerationNoGravity());
        final var loopTime = this.timer.get();
        this.timer.reset();
        this.speed = this.speed.add(this.gyro.getAccelerationNoGravity().mul(loopTime));
        Vec2D dis = this.speed.mul(loopTime);
        this.pos = this.pos.add(dis);
        return this;
    }

    public Vec2D getPosition() {
        return pos;
    }

    public Vec2D getSpeed() {
        return this.speed;
    }

    public double getYaw() {
        return this.gyro.getYaw() - this.zero;
    }

    @Override
    public String key() {
        return "inav";
    }

    public interface Gyro {
        double getYaw();

        Vec2D getAccelerationNoGravity();
    }
}