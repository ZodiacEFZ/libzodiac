package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The <i>CTRE MagEncoder</i> sensor.
 */
public final class MagEncoder {
    private static final double CTRE_MAG_ENCODER_UNIT = 2 * Math.PI / 4096;

    private final TalonSRX encoder;

    /**
     * Zero position of the sensor in raw unit (the 4096 unit).
     */
    public double zero = 0;

    public MagEncoder(int can_id, double zero) {
        this(can_id);
        this.zero = zero;
    }

    public MagEncoder(int can_id) {
        this.encoder = new TalonSRX(can_id);
        this.encoder.configFactoryDefault();
        this.encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    }

    /**
     * Configure the zero position of the encoder.
     *
     * @param zero the zero position in raw units.
     */
    public void setZero(double zero) {
        this.zero = zero;
    }

    public void reset() {
        this.encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.encoder.setSelectedSensorPosition(0);
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(this.getRadians());
    }

    public double getRadians() {
        return (this.encoder.getSelectedSensorPosition() - this.zero) * CTRE_MAG_ENCODER_UNIT;
    }
}
