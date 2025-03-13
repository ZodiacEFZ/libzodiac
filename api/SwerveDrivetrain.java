package frc.libzodiac.api;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.libzodiac.util.GameUtil;
import frc.libzodiac.util.Maths;
import frc.libzodiac.util.Rotation2dSupplier;
import frc.libzodiac.util.Translation2dSupplier;

import java.util.Collection;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public interface SwerveDrivetrain extends Drivetrain, Sendable {
    /**
     * Zeroes the heading of the robot.
     */
    void zeroHeading();

    boolean getFieldCentric();

    void setFieldCentric(boolean fieldCentric);

    void toggleFieldCentric();

    boolean getDirectAngle();

    void setDirectAngle(boolean directAngle);

    void toggleDirectAngle();

    boolean getSlowMode();

    void setSlowMode(boolean slowMode);

    void toggleSlowMode();

    double calculateRotation(Rotation2dSupplier headingSupplier);

    Collection<TalonFX> getTalonFXMotors();

    void setTargetHeading(Rotation2d targetHeading);

    default void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        this.drive(this.calculateChassisSpeeds(translation, rotation), fieldRelative);
    }

    private void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        if (fieldRelative) {
            this.driveFieldCentric(chassisSpeeds);
        } else {
            this.driveRobotCentric(chassisSpeeds);
        }
    }

    ChassisSpeeds calculateChassisSpeeds(Translation2d translation, double rotation);

    default void driveFieldCentric(ChassisSpeeds speeds) {
        this.driveRobotCentric(
                ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getYawRelative()));
    }

    /**
     * Returns the current yaw of the robot. The yaw of 0 is the robot facing directly away from
     * your alliance station wall.
     *
     * @return The current yaw.
     */
    default Rotation2d getYawRelative() {
        return GameUtil.toAllianceRelativeYaw(this.getYaw());
    }

    /**
     * Returns the current yaw of the robot. The yaw of 0 is the robot facing the red alliance
     * wall.
     *
     * @return The current yaw.
     */
    default Rotation2d getYaw() {
        return this.getPose().getRotation();
    }

    default Command getDriveCommand(Supplier<ChassisSpeeds> directAngle,
                                    Supplier<ChassisSpeeds> angularVelocity,
                                    BooleanSupplier driveDirectAngle,
                                    BooleanSupplier fieldRelative) {
        return run(() -> this.drive(
                driveDirectAngle.getAsBoolean() ? directAngle.get() : angularVelocity.get(),
                fieldRelative.getAsBoolean()));
    }

    class InputStream implements Supplier<ChassisSpeeds> {
        private final SwerveDrivetrain drivetrain;
        private final Translation2dSupplier translation;
        private double deadband = 0;
        private RotationType rotationType = RotationType.NONE;
        private Rotation2dSupplier heading;
        private DoubleSupplier rotation;

        public InputStream(SwerveDrivetrain drivetrain, Translation2dSupplier translation) {
            this.drivetrain = drivetrain;
            this.translation = translation;
        }

        @Override
        public ChassisSpeeds get() {
            var translation = Maths.squareTranslation(
                    Maths.applyDeadband(this.translation.get(), this.deadband));
            var rotation = switch (this.rotationType) {
                case HEADING -> this.drivetrain.calculateRotation(this.heading);
                case ROTATION -> Maths.square(
                        MathUtil.applyDeadband(this.rotation.getAsDouble(), this.deadband));
                case NONE -> 0;
            };
            return this.drivetrain.calculateChassisSpeeds(translation, rotation);
        }

        public InputStream rotation(DoubleSupplier rotation) {
            this.rotation = rotation;
            this.rotationType = RotationType.ROTATION;
            return this;
        }

        public InputStream deadband(double deadband) {
            this.deadband = deadband;
            return this;
        }

        public InputStream heading(Rotation2dSupplier heading) {
            this.heading = heading;
            this.rotationType = RotationType.HEADING;
            return this;
        }

        enum RotationType {
            HEADING,
            ROTATION,
            NONE
        }
    }
}
