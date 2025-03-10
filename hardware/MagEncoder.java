package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.libzodiac.api.Encoder;

/**
 * The <i>CTRE MagEncoder</i>.
 */
public final class MagEncoder implements Encoder {
    /**
     * The unit of the CTRE MagEncoder.
     */
    private static final AngleUnit CTRE_MAG_ENCODER_UNIT = Units.derive(Units.Rotations)
                                                                .splitInto(4096)
                                                                .named("MagEncoderUnit")
                                                                .symbol("")
                                                                .make();

    /**
     * The encoder.
     */
    private final TalonSRX encoder;

    /**
     * Zero position of the sensor in raw unit (4096 per rotation).
     */
    public double zero = 0;

    /**
     * Construct a new <i>MagEncoder</i>.
     *
     * @param can_id the CAN ID of the encoder.
     * @param zero   the zero position in raw units.
     */
    public MagEncoder(int can_id, double zero) {
        this.encoder = new TalonSRX(can_id);
        this.encoder.configFactoryDefault();
        this.encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        this.zero = zero;
    }

    /**
     * Construct a new <i>MagEncoder</i>.
     *
     * @param can_id the CAN ID of the encoder.
     */
    public MagEncoder(int can_id) {
        this.encoder = new TalonSRX(can_id);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.encoder.setInverted(inverted);
    }

    public void setContinuous(boolean continuous) {
        this.encoder.configFeedbackNotContinuous(!continuous, 0);
    }

    public void resetAbsoluteToRelative() {
        this.setPosition(this.get());
    }

    @Override
    public void setZero(double zero) {
        this.zero = zero;
    }

    @Override
    public void setPosition(Angle position) {
        this.zero = 0;
        this.encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.encoder.setSelectedSensorPosition(position.in(CTRE_MAG_ENCODER_UNIT));
    }

    @Override
    public void reset() {
        this.zero = 0;
        this.encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.encoder.setSelectedSensorPosition(0);
    }

    public void setSensor(FeedbackDevice sensor) {
        this.encoder.configSelectedFeedbackSensor(sensor);
    }

    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d(this.get());
    }

    @Override
    public Angle get() {
        return CTRE_MAG_ENCODER_UNIT.of(this.encoder.getSelectedSensorPosition() - this.zero);
    }
}
