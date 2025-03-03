package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Utility for constructing {@link Supplier<Translation2d>} with two {@link DoubleSupplier}s as x and y dimension.
 */
public record Translation2dSupplier(DoubleSupplier x, DoubleSupplier y) implements Supplier<Translation2d> {
    @Override
    public Translation2d get() {
        return new Translation2d(this.x.getAsDouble(), this.y.getAsDouble());
    }
}
