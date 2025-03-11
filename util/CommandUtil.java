package frc.libzodiac.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * A utility class for commands
 */
public class CommandUtil {
    /**
     * Rumbles the controller for a specified duration and intensity
     *
     * @param controller The controller to rumble
     * @param intensity  The intensity of the rumble
     * @param duration   The duration of the rumble
     */
    public static void rumbleController(GenericHID controller, double intensity, double duration) {
        rumbleControllerCommand(controller, intensity, duration).schedule();
    }

    /**
     * Creates a command that rumbles the controller for a specified duration and intensity
     *
     * @param controller The controller to rumble
     * @param intensity  The intensity of the rumble
     * @param duration   The duration of the rumble
     *
     * @return The command that rumbles the controller
     */
    public static Command rumbleControllerCommand(GenericHID controller, double intensity,
                                                  double duration) {
        return Commands.runOnce(
                               () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, intensity))
                       .repeatedly()
                       .withTimeout(duration)
                       .finallyDo(() -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0))
                       .ignoringDisable(true);
    }
}
