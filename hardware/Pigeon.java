package frc.libzodiac.hardware;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.libzodiac.api.Gyro;

public class Pigeon implements Gyro {
    final Pigeon2 pigeon;

    public Pigeon(int id) {
        this.pigeon = new Pigeon2(id);
        this.factoryDefault();
    }

    public Angle getYaw() {
        return this.pigeon.getYaw().getValue();
    }

    public void factoryDefault() {
        this.pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    @Override
    public Rotation2d getRotation2d() {
        return this.pigeon.getRotation2d();
    }

    @Override
    public AngularVelocity getYawAngularVelocity() {
        return this.pigeon.getAngularVelocityZWorld().getValue();
    }

    @Override
    public void reset() {
        this.pigeon.reset();
    }
}
