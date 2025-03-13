package frc.libzodiac.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.libzodiac.api.Encoder;

public class FakeEncoder implements Encoder {
    private Angle position = Units.Radian.of(0);

    @Override
    public void setInverted(boolean inverted) {
    }

    @Override
    public void setZero(double zero) {
    }

    @Override
    public void setPosition(Angle position) {
        this.position = position;
    }

    @Override
    public void reset() {
        this.position = Units.Radian.of(0);
    }

    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d(this.get());
    }

    @Override
    public Angle get() {
        return this.position;
    }
}
