package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Utility for constructing {@link Supplier<Translation2d>} with two {@link DoubleSupplier}s as x
 * and y dimension.
 */
public record Translation2dSupplier(DoubleSupplier x, DoubleSupplier y)
        implements Supplier<Translation2d> {
    public Translation2dSupplier(Supplier<Rotation2d> rotation, Supplier<Double> norm) {
        this(() -> Math.cos(rotation.get().getRadians()) * norm.get(),
             () -> Math.sin(rotation.get().getRadians()) * norm.get());
    }

    @Override
    public Translation2d get() {
        return new Translation2d(this.x.getAsDouble(), this.y.getAsDouble());
    }
}
