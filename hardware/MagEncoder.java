package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

/**
 * The <i>CTRE MagEncoder</i> sensor.
 */
public final class MagEncoder {
    private static final AngleUnit CTRE_MAG_ENCODER_UNIT = Units.derive(Units.Rotations).splitInto(4096).named("MagEncoderUnit").symbol("")
            .make();

    private final TalonSRX encoder;

    /**
     * Zero position of the sensor in raw unit (the 4096 unit).
     */
    public double zero = 0;

    public MagEncoder(int can_id, double zero) {
        this.encoder = new TalonSRX(can_id);
        this.encoder.configFactoryDefault();
        this.encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        this.zero = zero;
    }

    public MagEncoder(int can_id) {
        this.encoder = new TalonSRX(can_id);
    }

    public void setInverted(boolean inverted) {
        this.encoder.setSensorPhase(inverted);
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
        this.zero = 0;
        this.encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.encoder.setSelectedSensorPosition(0);
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(this.get());
    }

    public Angle get() {
        return CTRE_MAG_ENCODER_UNIT.of(this.encoder.getSelectedSensorPosition() - this.zero);
    }
}
