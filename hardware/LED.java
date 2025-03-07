package frc.libzodiac.hardware;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class LED {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    public LED(int port, int length) {
        led = new AddressableLED(port);
        led.setLength(length);
        buffer = new AddressableLEDBuffer(length);
        led.setData(buffer);
        led.start();
    }

    public Command getApplyPatternCommand(LEDPattern pattern) {
        return runOnce(() -> this.apply(pattern)).ignoringDisable(true);
    }

    public void apply(LEDPattern pattern) {
        pattern.applyTo(buffer);
        led.setData(buffer);
    }
}
