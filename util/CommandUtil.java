package frc.libzodiac.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandUtil {
    public static Command rumbleController(GenericHID controller, double intensity, double duration) {
        return Commands.runOnce(() -> controller.setRumble(GenericHID.RumbleType.kBothRumble, intensity)).repeatedly()
                .withTimeout(duration).finallyDo(() -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0));
    }
}