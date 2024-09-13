package frc.libzodiac.hardware;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.libzodiac.ZInertialNavigation;
import frc.libzodiac.Zensor;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.ui.Axis;
import frc.libzodiac.util.Vec2D;

import java.security.InvalidParameterException;

public class Pigeon implements Zensor, ZInertialNavigation.Gyro, ZmartDash {
    protected Pigeon2 pigeon;

    public Pigeon(int can_id) {
        this.pigeon = new Pigeon2(can_id);
    }

    public Axis yaw() {
        return new Axis(() -> Math.toRadians(this.pigeon.getYaw().refresh().getValue()));
    }

    public Axis pitch() {
        return new Axis(() -> Math.toRadians(this.pigeon.getPitch().refresh().getValue()));
    }

    public Axis roll() {
        return new Axis(() -> Math.toRadians(this.pigeon.getRoll().refresh().getValue()));
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
    public String key() {
        return "Pigeon(" + this.pigeon.getDeviceID() + ")";
    }

    @Override
    public double getYaw() {
        return this.yaw().get();
    }

    @Override
    public Vec2D getAccelerationNoGravity() {
        return new Vec2D(this.pigeon.getAccelerationX().refresh().getValue() - this.pigeon.getGravityVectorX().refresh().getValue(), this.pigeon.getAccelerationY().refresh().getValue() - this.pigeon.getGravityVectorY().refresh().getValue()).mul(9.8);
    }
}