package frc.libzodiac.unused.hardware;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.Units;
import frc.libzodiac.unused.api.Zensor;
import frc.libzodiac.unused.util.Axis;
import frc.libzodiac.unused.util.Lazy;

import java.security.InvalidParameterException;

/**
 * The <i>Pigeon 2</i> gyrometer.
 */
public class Pigeon implements Zensor {
    protected final Lazy<Pigeon2> pigeon;

    public Pigeon(int can_id) {
        this.pigeon = Lazy.with(() -> new Pigeon2(can_id));
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
        return Axis.with(() -> this.pigeon.get().getYaw().refresh().getValue().in(Units.Radian));
    }

    /**
     * The pitch output, or deviation form the +x axis on the xz plane.
     *
     * @return output in radians
     */
    public Axis pitch() {
        return Axis.with(() -> this.pigeon.get().getPitch().refresh().getValue().in(Units.Radian));
    }

    /**
     * The pitch output, or deviation form the +z axis on the yz plane.
     *
     * @return output in radians
     */
    public Axis roll() {
        return Axis.with(() -> this.pigeon.get().getRoll().refresh().getValue().in(Units.Radian));
    }

    public Pigeon2 raw_api() {
        return this.pigeon.get();
    }
}
