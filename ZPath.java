package frc.libzodiac;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;

public class ZPath {
    Trajectory trajectory = new Trajectory();

    public ZPath(String json) {
        try {
            Path path = Filesystem.getDeployDirectory().toPath().resolve(json);
            trajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + json, ex.getStackTrace());
        }
    }
}
