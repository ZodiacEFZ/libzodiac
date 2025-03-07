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
     *
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
     *
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
     *
     * @return The translation with the deadband applied
     */
    public static Translation2d applyDeadband(Translation2d translation, double deadband) {
        var distance = translation.getNorm();
        if (distance < deadband) {
            return new Translation2d();
        }
        return new Translation2d(distance, translation.getAngle());
    }

    /**
     * Limits the translation to a norm of 1
     *
     * @param translation The translation to limit
     *
     * @return The limited translation
     */
    public static Translation2d limitTranslation(Translation2d translation, double limit) {
        var norm = translation.getNorm();
        return norm > limit ? translation.times(limit / norm) : translation;
    }

    /**
     * Solve an angle in a certain triangle using law of cosine.
     *
     * @param opposite  length of opposite edge
     * @param adjacent1 length of an adjacent edge
     * @param adjacent2 length of an adjacent edge
     *
     * @return the angle in radians
     */
    public static double resolveAngle(double opposite, double adjacent1, double adjacent2) {
        final var cos =
                (opposite * opposite - adjacent1 * adjacent1 - adjacent2 * adjacent2) / (2 * adjacent1 * adjacent2);
        return Math.acos(cos);
    }

    /**
     * Solve the opposite edge of certain angle using law of cosine.
     *
     * @param angle     the angle in radians
     * @param adjacent1 length of an adjacent edge
     * @param adjacent2 length of an adjacent edge
     *
     * @return length of the opposite edge
     */
    public static double resolveEdge(double angle, double adjacent1, double adjacent2) {
        final var sqr = adjacent1 * adjacent1 + adjacent2 * adjacent2 - 2 * adjacent1 * adjacent2 * Math.cos(angle);
        return Math.sqrt(sqr);
    }
}
