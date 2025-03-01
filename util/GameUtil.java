package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class GameUtil {
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }

    public static boolean isBlueAlliance() {
        return !isRedAlliance();
    }

    public static Rotation2d toAllianceRelativeYaw(Rotation2d pose2dYaw) {
        return isRedAlliance() ? pose2dYaw.rotateBy(new Rotation2d(Math.PI)) : pose2dYaw;
    }

    public static Rotation2d toPose2dYaw(Rotation2d allianceRelativeYaw) {
        return isRedAlliance() ? allianceRelativeYaw.rotateBy(new Rotation2d(Math.PI)) : allianceRelativeYaw;
    }
}
