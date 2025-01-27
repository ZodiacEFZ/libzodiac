package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Translation2d;

public class Maths {
    public static Translation2d cubeTranslation(Translation2d translation) {
        return new Translation2d(cube(translation.getNorm()), translation.getAngle());
    }

    public static double cube(double v) {
        return v * v * v;
    }

    public static Translation2d applyDeadband(Translation2d translation, double deadband) {
        var distance = translation.getNorm();
        return new Translation2d(distance < deadband ? 0 : distance, translation.getAngle());
    }
}
