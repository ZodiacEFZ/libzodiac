package frc.libzodiac.hardware;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.libzodiac.api.Gyro;

/**
 * The CTRE Pigeon 2 gyro.
 */
public final class Pigeon implements Gyro {
    final Pigeon2 pigeon;

    /**
     * Construct a new Pigeon.
     *
     * @param id The CAN ID of the Pigeon 2.
     */
    public Pigeon(int id) {
        this.pigeon = new Pigeon2(id);
        this.factoryDefault();
    }

    /**
     * Factory default the gyro.
     */
    public void factoryDefault() {
        this.pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    @Override
    public void reset() {
        this.pigeon.reset();
    }

    @Override
    public Angle getYaw() {
        return this.pigeon.getYaw().getValue();
    }

    @Override
    public Rotation2d getRotation2d() {
        return this.pigeon.getRotation2d();
    }

    @Override
    public Rotation3d getRotation3d() {
        return this.pigeon.getRotation3d();
    }

    @Override
    public AngularVelocity getYawAngularVelocity() {
        return this.pigeon.getAngularVelocityZWorld().getValue();
    }
}
