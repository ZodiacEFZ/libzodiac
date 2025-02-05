package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Translation2dSupplier implements Supplier<Translation2d> {
    final DoubleSupplier x;
    final DoubleSupplier y;

    public Translation2dSupplier(DoubleSupplier x, DoubleSupplier y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public Translation2d get() {
        return new Translation2d(this.x.getAsDouble(), this.y.getAsDouble());
    }
}
