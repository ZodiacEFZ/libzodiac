package frc.libzodiac.hardware;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.libzodiac.api.Encoder;

public class CANCoder implements Encoder {
    private final CANcoder encoder;
    public double zero = 0;

    public CANCoder(int deviceID) {
        this.encoder = new CANcoder(deviceID);
    }

    public CANCoder(int deviceID, double zero) {
        this.encoder = new CANcoder(deviceID);
        this.encoder.getConfigurator().apply(new CANcoderConfiguration());
        this.setZero(zero);
    }

    @Override
    public void setInverted(boolean inverted) {
        var config = new MagnetSensorConfigs();
        this.encoder.getConfigurator().refresh(config);
        this.encoder.getConfigurator()
                    .apply(config.withSensorDirection(
                            inverted ? SensorDirectionValue.Clockwise_Positive :
                                    SensorDirectionValue.CounterClockwise_Positive));
    }

    @Override
    public void setZero(double zero) {
        var config = new MagnetSensorConfigs();
        this.encoder.getConfigurator().refresh(config);
        this.encoder.getConfigurator().apply(config.withMagnetOffset(-zero));
    }

    @Override
    public void setPosition(Angle position) {
        this.encoder.setPosition(position);
    }

    @Override
    public void reset() {
        this.encoder.setPosition(0);
    }

    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d(this.get());
    }

    @Override
    public Angle get() {
        return this.encoder.getPosition().getValue();
    }
}
