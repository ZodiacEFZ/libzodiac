package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A class to hold mathematical functions
 */
public class Maths {
    /**
     * Squares the norm of the translation
     *
     * @param translation The translation to square
     * @return The squared translation
     */
    public static Translation2d squareTranslation(Translation2d translation) {
        if (translation.getNorm() == 0) {
            return translation;
        }
        return new Translation2d(square(translation.getNorm()), translation.getAngle());
    }

    /**
     * Squares a value
     *
     * @param v The value to square
     * @return The squared value
     */
    public static double square(double v) {
        return v * Math.abs(v);
    }

    /**
     * Applies a deadband to a translation
     *
     * @param translation The translation to apply the deadband to
     * @param deadband    The deadband to apply
     * @return The translation with the deadband applied
     */
    public static Translation2d applyDeadband(Translation2d translation, double deadband) {
        var distance = translation.getNorm();
        if (distance < deadband) {
            return new Translation2d();
        }
        return new Translation2d(distance, translation.getAngle());
    }
}
