package frc.libzodiac.hardware;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.libzodiac.api.Gyro;

/**
 * The NavX gyro on the roboRIO.
 */
public final class NavX implements Gyro {
    final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

    @Override
    public Angle getYaw() {
        return Units.Degrees.of(this.navX.getAngle());
    }

    @Override
    public Rotation2d getRotation2d() {
        return this.navX.getRotation2d();
    }

    @Override
    public Rotation3d getRotation3d() {
        return this.navX.getRotation3d();
    }

    @Override
    public AngularVelocity getYawAngularVelocity() {
        return Units.DegreesPerSecond.of(this.navX.getRawGyroZ());
    }

    @Override
    public void reset() {
        this.navX.reset();
    }
}
