package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Translation2d;

public class Maths {
    public static Translation2d squareTranslation(Translation2d translation) {
        if (translation.getNorm() == 0) {
            return translation;
        }
        return new Translation2d(square(translation.getNorm()), translation.getAngle());
    }

    public static double square(double v) {
        return v * Math.abs(v);
    }

    public static Translation2d applyDeadband(Translation2d translation, double deadband) {
        var distance = translation.getNorm();
        if (distance < deadband) {
            return new Translation2d();
        }
        return new Translation2d(distance, translation.getAngle());
    }
}
