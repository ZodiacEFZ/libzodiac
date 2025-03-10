package frc.libzodiac.hardware;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

public class LEDController extends SubsystemBase {
    private static LEDController INSTANCE;
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final Map<String, LED> leds = new HashMap<>();

    private LEDController(int port, int length) {
        this.led = new AddressableLED(port);
        this.buffer = new AddressableLEDBuffer(length);
        this.led.setLength(length);
        this.led.setData(buffer);
        this.led.start();
    }

    public static void initInstance(int port, int length) {
        if (INSTANCE == null) {
            INSTANCE = new LEDController(port, length);
        }
    }

    public static LEDController getInstance() {
        return INSTANCE;
    }

    public LED add(String name, int start, int end) {
        var led = new LED(start, end);
        this.leds.put(name, led);
        return led;
    }

    public LED get(String name) {
        return this.leds.get(name);
    }

    public void applyAll(LEDPattern pattern) {
        this.leds.values().forEach(led -> led.apply(pattern));
    }

    @Override
    public void periodic() {
        this.led.setData(buffer);
    }

    public static class LED extends SubsystemBase {
        private final AddressableLEDBufferView bufferView;

        public LED(int start, int end) {
            this.bufferView = LEDController.INSTANCE.buffer.createView(start, end);
            this.setDefaultCommand(this.getApplyPatternCommand(LEDPattern.rainbow(255, 255)
                                                                         .scrollAtAbsoluteSpeed(
                                                                                 Units.MetersPerSecond.of(0.5),
                                                                                 Units.Meters.of(1.0 / 60))));
        }

        public Command getApplyPatternCommand(LEDPattern pattern) {
            return runOnce(() -> this.apply(pattern)).ignoringDisable(true);
        }

        public void apply(LEDPattern pattern) {
            pattern.applyTo(this.bufferView);
        }
    }
}
