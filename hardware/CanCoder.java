package frc.libzodiac.hardware;

import com.ctre.phoenix6.hardware.CANcoder;
import frc.libzodiac.util.Lazy;

public class CanCoder {
    public static final double POSITION_RAW_UNIT = 2 * Math.PI;
    private final Lazy<CANcoder> encoder;
    public double zero = 0;

    public CanCoder(int can_id) {
        this.encoder = Lazy.with(() -> new CANcoder(can_id));
    }

    public CanCoder set_zero(double zero) {
        this.zero = zero;
        return this;
    }

    public double get() {
        return (this.encoder.get().getAbsolutePosition().refresh().getValue() - this.zero) * POSITION_RAW_UNIT;
    }
}
