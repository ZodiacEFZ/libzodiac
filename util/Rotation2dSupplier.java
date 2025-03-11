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
    final Supplier<Rotation2d> rotation2dSupplier;

    /**
     * Construct a new Rotation2dSupplier.
     *
     * @param x The x supplier.
     * @param y The y supplier.
     */
    public Rotation2dSupplier(DoubleSupplier x, DoubleSupplier y) {
        this.translation = new Translation2dSupplier(x, y);
        this.rotation2dSupplier = () -> this.translation.get().getAngle();
    }

    public Rotation2dSupplier(Supplier<Rotation2d> rotation2dSupplier, Supplier<Double> norm) {
        this.rotation2dSupplier = rotation2dSupplier;
        this.translation = new Translation2dSupplier(rotation2dSupplier, norm);
    }

    @Override
    public Rotation2d get() {
        return this.rotation2dSupplier.get();
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