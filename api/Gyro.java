package frc.libzodiac.api;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface Gyro {
    void reset();

    Rotation2d getRotation2d();

    AngularVelocity getYawAngularVelocity();
}
