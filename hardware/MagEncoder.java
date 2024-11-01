package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.libzodiac.util.Lazy;

public final class MagEncoder {
    public static final double POSITION_RAW_UNIT = 2 * Math.PI / 4096;
    private final Lazy<TalonSRX> encoder;
    public double zero = 0;

    public MagEncoder(int can_id) {
        this.encoder = new Lazy<>(() -> {
            var encoder = new TalonSRX(3);
            encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
            return encoder;
        });
    }

    public MagEncoder set_zero(double zero) {
        this.zero = zero;
        return this;
    }

    public double get() {
        return (this.encoder.get().getSelectedSensorPosition() - this.zero) * POSITION_RAW_UNIT;
    }
}
