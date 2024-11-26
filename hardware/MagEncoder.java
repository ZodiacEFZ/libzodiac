package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.libzodiac.util.Lazy;

/**
 * The <i>CTRE MagEncoder</i> sensor.
 */
public final class MagEncoder {

    /**
     * The <i>MagEncoder</i> use 4096 sensor units per rotation.
     */
    public static final double POSITION_RAW_UNIT = 2 * Math.PI / 4096;

    private final Lazy<TalonSRX> encoder;

    /**
     * Zero position of the sensor in raw unit (the 4096 unit).
     */
    public double zero = 0;

    private MagEncoder(int can_id) {
        this.encoder = new Lazy<>(() -> {
            var encoder = new TalonSRX(can_id);
            encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
            return encoder;
        });
    }

    /**
     * Bind to a <i>CTRE MagEncoder</i>.
     *
     * @param can_id id on the CAN bus
     */
    public static MagEncoder with(int can_id) {
        return new MagEncoder(can_id);
    }

    /**
     * Configure the zero position of the encoder.
     * 
     * @param zero the zero position in raw units.
     * @return self for chaining
     */
    public MagEncoder config_zero(double zero) {
        this.zero = zero;
        return this;
    }

    public double get() {
        return (this.encoder.get().getSelectedSensorPosition() - this.zero) * POSITION_RAW_UNIT;
    }
}
