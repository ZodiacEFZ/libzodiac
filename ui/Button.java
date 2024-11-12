package frc.libzodiac.ui;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

public class Button {
    private final BooleanSupplier status;
    private boolean prev = false;

    public Button(BooleanSupplier status) {
        this.status = status;
    }

    public boolean pressed() {
        final var curr = this.down();
        final var res = !this.prev && curr;
        this.prev = curr;
        return res;
    }

    public boolean down() {
        return this.status.getAsBoolean();
    }

    public boolean released() {
        final var curr = this.down();
        final var res = this.prev && !curr;
        this.prev = curr;
        return res;
    }

    public Button on_press(Command cmd) {
        this.trigger().onTrue(cmd);
        return this;
    }

    public Trigger trigger() {
        return new Trigger(this.status);
    }

    public Button on_release(Command cmd) {
        this.trigger().onFalse(cmd);
        return this;
    }

    public Button on_down(Command cmd) {
        this.trigger().whileTrue(cmd);
        return this;
    }

    public Button on_up(Command cmd) {
        this.trigger().whileFalse(cmd);
        return this;
    }
}
