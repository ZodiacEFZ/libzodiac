package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Rotation2dSupplier implements Supplier<Rotation2d> {
    final DoubleSupplier x;
    final DoubleSupplier y;

    public Rotation2dSupplier(DoubleSupplier x, DoubleSupplier y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public Rotation2d get() {
        return new Rotation2d(this.x.getAsDouble(), this.y.getAsDouble());
    }
}