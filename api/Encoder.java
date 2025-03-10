package frc.libzodiac.api;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public interface Encoder {
    /**
     * Set the phase of the encoder.
     *
     * @param inverted the phase of the encoder.
     */
    void setInverted(boolean inverted);

    /**
     * Configure the zero position of the encoder.
     *
     * @param zero the zero position in raw units.
     */
    void setZero(double zero);

    void setPosition(Angle position);

    /**
     * Reset the encoder.
     */
    void reset();

    /**
     * Get the angle of the encoder in {@link Rotation2d}.
     *
     * @return the angle of the encoder in  {@link Rotation2d}.
     */
    Rotation2d getRotation2d();

    /**
     * Get the angle of the encoder.
     *
     * @return the angle of the encoder.
     */
    Angle get();
}
