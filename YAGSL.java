package frc.libzodiac;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.ui.Xbox;
import frc.libzodiac.util.Lazy;
import frc.libzodiac.util.Vec2;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import java.io.File;
import java.io.IOException;

/**
 * Re-export entry point to the YAGSL swerve library.
 */
public final class YAGSL extends SubsystemBase {

    private final File path;
    private double max_speed;
    private Lazy<SwerveDrive> drive = Lazy.with(this::init);
    public boolean headless = false;

    private YAGSL(String path) {
        this.path = new File(Filesystem.getDeployDirectory(), path);
    }

    public static YAGSL with(String path) {
        return new YAGSL(path);
    }

    public YAGSL config_max_speed(double speed) {
        this.max_speed = speed;
        return this;
    }

    private SwerveDrive init() {
        try {
            return new SwerveParser(this.path).createSwerveDrive(this.max_speed);
        } catch (IOException err) {
            System.err.println(err);
            System.exit(-1);
        }
        return null;
    }

    public void drive(ChassisSpeeds req) {
        if (this.headless)
            this.drive.get().driveFieldOriented(req);
        else
            this.drive.get().drive(req);
    }

    public SwerveDrive raw_api() {
        return this.drive.get();
    }

    public void switch_headless() {
        this.headless = !this.headless;
    }

    public Command drive_with(Xbox controller) {
        var cmd = Commands.run(() -> {
            var dx = controller.lx().get();
            var dy = controller.ly().get();
            var da = controller.rx().get();
            var req = new ChassisSpeeds(dx, dy, da);
            if (this.headless)
                this.drive.get().driveFieldOriented(req);
            else
                this.drive.get().drive(req);
        }, this);
        return cmd;
    }

}
