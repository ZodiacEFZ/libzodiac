package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.libzodiac.ZmartDash;

public final class MagEncoder implements ZmartDash {
    public static final double POSITION_RAW_UNIT = 2 * Math.PI / 4096;
    private final TalonSRX encoder;
    public double zero = 0;

    public MagEncoder(int can_id) {
        this.encoder = new TalonSRX(can_id);
        this.encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    }

    public MagEncoder set_zero(double zero) {
        this.zero = zero;
        return this;
    }

    public double get() {
        return (this.encoder.getSelectedSensorPosition() - this.zero) * POSITION_RAW_UNIT;
    }

    @Override
    public String key() {
        return "MagEncoder(" + this.encoder.getDeviceID() + ")";
    }
}
