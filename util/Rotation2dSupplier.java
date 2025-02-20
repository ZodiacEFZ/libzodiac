package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A supplier for a Rotation2d.
 */
public class Rotation2dSupplier implements Supplier<Rotation2d> {
    /**
     * The translation supplier.
     */
    final Translation2dSupplier translation;

    /**
     * Construct a new Rotation2dSupplier.
     *
     * @param x The x supplier.
     * @param y The y supplier.
     */
    public Rotation2dSupplier(DoubleSupplier x, DoubleSupplier y) {
        this.translation = new Translation2dSupplier(x, y);
    }

    @Override
    public Rotation2d get() {
        return this.translation.get().getAngle();
    }

    /**
     * Get the translation.
     *
     * @return The translation.
     */
    public Translation2d asTranslation() {
        return this.translation.get();
    }
}