package frc.libzodiac.hardware;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.libzodiac.ZInertialNavigation;
import frc.libzodiac.Zensor;
import frc.libzodiac.ui.Axis;
import frc.libzodiac.util.Lazy;
import frc.libzodiac.util.Vec2D;

import java.security.InvalidParameterException;

public class Pigeon implements Zensor, ZInertialNavigation.Gyro {
    protected final Lazy<Pigeon2> pigeon;

    public Pigeon(int can_id) {
        this.pigeon = new Lazy<>(() -> new Pigeon2(can_id));
    }

    public Axis yaw() {
        return new Axis(() -> Math.toRadians(this.pigeon.get().getYaw().refresh().getValue()));
    }

    public Axis pitch() {
        return new Axis(() -> Math.toRadians(this.pigeon.get().getPitch().refresh().getValue()));
    }

    public Axis roll() {
        return new Axis(() -> Math.toRadians(this.pigeon.get().getRoll().refresh().getValue()));
    }

    @Override
    public double get() {
        return this.yaw().get();
    }

    @Override
    public double get(String value) {
        return switch (value.trim().toLowerCase()) {
            case "yaw" -> this.yaw().get();
            case "pitch" -> this.pitch().get();
            case "roll" -> this.roll().get();
            default -> throw new InvalidParameterException(value);
        };
    }

    @Override
    public double getYaw() {
        return this.yaw().get();
    }

    @Override
    public Vec2D getAccelerationNoGravity() {
        return new Vec2D(
                this.pigeon.get().getAccelerationX().refresh().getValue()
                        - this.pigeon.get().getGravityVectorX().refresh().getValue(),
                this.pigeon.get().getAccelerationY().refresh().getValue()
                        - this.pigeon.get().getGravityVectorY().refresh().getValue())
                .mul(9.8);
    }
}