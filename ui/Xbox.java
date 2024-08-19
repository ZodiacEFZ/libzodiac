package frc.libzodiac.ui;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

public final class Xbox {
    public final int port;
    private final XboxController xbox;

    public Xbox(int port) {
        this.port = port;
        this.xbox = new XboxController(this.port);
    }

    public Axis lx() {
        return new Axis(this.xbox::getLeftX);
    }

    public Axis ly() {
        return new Axis(this.xbox::getLeftY);
    }

    public Axis rx() {
        return new Axis(this.xbox::getRightX);
    }

    public Axis ry() {
        return new Axis(this.xbox::getRightY);
    }

    public Axis lt() {
        return new Axis(this.xbox::getLeftTriggerAxis);
    }

    public Axis rt() {
        return new Axis(this.xbox::getRightTriggerAxis);
    }

    public Button lb() {
        return new Button(this.xbox::getLeftBumper);
    }

    public Button rb() {
        return new Button(this.xbox::getRightBumper);
    }

    public Button up_pov() {
        final var pov = this.xbox.getPOV();
        return new Button(() -> pov < 45 || pov >= 315);
    }

    public Button right_pov() {
        final var pov = this.xbox.getPOV();
        return new Button(() -> pov >= 45 && pov < 135);
    }

    public Button down_pov() {
        final var pov = this.xbox.getPOV();
        return new Button(() -> pov >= 135 && pov < 225);
    }

    public Button left_pov() {
        final var pov = this.xbox.getPOV();
        return new Button(() -> pov >= 225 && pov < 315);
    }

    public Button a() {
        return new Button(this.xbox::getAButton);
    }

    public Button b() {
        return new Button(this.xbox::getBButton);
    }

    public Button x() {
        return new Button(this.xbox::getXButton);
    }

    public Button y() {
        return new Button(this.xbox::getYButton);
    }

    public Xbox begin_rumble() {
        this.xbox.setRumble(RumbleType.kBothRumble, 0.5);
        return this;
    }
}
