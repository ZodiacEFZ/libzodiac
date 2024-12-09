package frc.libzodiac;

import edu.wpi.first.wpilibj.Filesystem;
import frc.libzodiac.ui.Axis2D;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;

public class YAGSL extends Zubsystem {
    private static final double MAXIMUM_SPEED = 5;
    SwerveDrive swerveDrive = null;

    public YAGSL() {
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(MAXIMUM_SPEED);
        } catch (IOException ignored) {
        }

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        
    }
}

