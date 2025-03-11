package frc.libzodiac.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class GameUtil {
    /**
     * Get whether the robot is on the blue alliance.
     *
     * @return Whether the robot is on the blue alliance.
     */
    public static boolean isBlueAlliance() {
        return !isRedAlliance();
    }

    /**
     * Get whether the robot is on the red alliance.
     *
     * @return Whether the robot is on the red alliance.
     */
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
               DriverStation.Alliance.Red;
    }

    /**
     * Get alliance-relative rotation from Pose2d rotation. The yaw of 0 is the robot facing the
     * opponents' alliance wall.
     *
     * @return The alliance-relative rotation.
     */
    public static Rotation2d toAllianceRelativeYaw(Rotation2d pose2dRotation) {
        return isRedAlliance() ? pose2dRotation.rotateBy(new Rotation2d(Math.PI)) : pose2dRotation;
    }

    /**
     * Get Pose2d rotation from alliance-relative rotation. The yaw of 0 is the robot facing the red
     * alliance wall.
     *
     * @return The robot-relative yaw.
     */
    public static Rotation2d toPose2dYaw(Rotation2d allianceRelativeRotation) {
        return isRedAlliance() ? allianceRelativeRotation.rotateBy(new Rotation2d(Math.PI)) :
                       allianceRelativeRotation;
    }
}
