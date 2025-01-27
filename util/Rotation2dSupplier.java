package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Rotation2dSupplier implements Supplier<Rotation2d> {
    final Translation2dSupplier translation;

    public Rotation2dSupplier(DoubleSupplier x, DoubleSupplier y) {
        this.translation = new Translation2dSupplier(x, y);
    }

    @Override
    public Rotation2d get() {
        return this.translation.get().getAngle();
    }

    public Translation2d asTranslation() {
        return this.translation.get();
    }
}