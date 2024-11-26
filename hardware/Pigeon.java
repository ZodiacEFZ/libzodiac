package frc.libzodiac.hardware;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.libzodiac.ZInertialNavigation;
import frc.libzodiac.api.Zensor;
import frc.libzodiac.ui.Axis;
import frc.libzodiac.util.Lazy;
import frc.libzodiac.util.Vec2;

import java.security.InvalidParameterException;

/**
 * The <i>Pigeon 2</i> gyrometer.
 */
public class Pigeon implements Zensor, ZInertialNavigation.Gyro {
    protected final Lazy<Pigeon2> pigeon;

    public Pigeon(int can_id) {
        this.pigeon = new Lazy<>(() -> new Pigeon2(can_id));
    }

    /**
     * Bind to a <i>Pigeon</i>.
     *
     * @param can_id id on the CAN bus
     */
    public static Pigeon with(int can_id) {
        return new Pigeon(can_id);
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
    public double get() {
        return this.yaw().get();
    }

    /**
     * The yaw output, or deviation form the +x axis on the xy plane.
     * 
     * @return output in radians
     */
    public Axis yaw() {
        return Axis.with(() -> Math.toRadians(this.pigeon.get().getYaw().refresh().getValue()));
    }

    /**
     * The pitch output, or deviation form the +x axis on the xz plane.
     * 
     * @return output in radians
     */
    public Axis pitch() {
        return Axis.with(() -> Math.toRadians(this.pigeon.get().getPitch().refresh().getValue()));
    }

    /**
     * The pitch output, or deviation form the +z axis on the yz plane.
     * 
     * @return output in radians
     */
    public Axis roll() {
        return Axis.with(() -> Math.toRadians(this.pigeon.get().getRoll().refresh().getValue()));
    }

    @Override
    public double getYaw() {
        return this.yaw().get();
    }

    @Override
    public Vec2 getAccelerationNoGravity() {
        return new Vec2(
                this.pigeon.get().getAccelerationX().refresh().getValue()
                        - this.pigeon.get().getGravityVectorX().refresh().getValue(),
                this.pigeon.get().getAccelerationY().refresh().getValue()
                        - this.pigeon.get().getGravityVectorY().refresh().getValue())
                .mul(9.8);
    }
}