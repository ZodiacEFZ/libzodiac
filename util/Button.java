package frc.libzodiac.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

/**
 * A binary, or on-off input, which could either be yielded by sensors or
 * controllers.
 */
public class Button {

    /**
     * Internal access to raw data source.
     */
    private final BooleanSupplier raw_input;

    /**
     * Whether the button was previously held down.
     */
    private boolean prev = false;

    private Button(BooleanSupplier raw_input) {
        this.raw_input = raw_input;
    }

    /**
     * Bind to a raw data source.
     * 
     * @param raw_input the raw data input
     * @return new
     */
    public static Button with(BooleanSupplier raw_input) {
        return new Button(raw_input);
    }

    /**
     * Whether the button was just pressed after last query.
     * 
     * @return the status change
     */
    public boolean pressed() {
        final var curr = this.down();
        final var res = !this.prev && curr;
        this.prev = curr;
        return res;
    }

    /**
     * Whether the button is currently held down.
     * 
     * @return the status
     */
    public boolean down() {
        return this.raw_input.getAsBoolean();
    }

    /**
     * Whether the button was just released after last query.
     * 
     * @return the status change
     */
    public boolean released() {
        final var curr = this.down();
        final var res = this.prev && !curr;
        this.prev = curr;
        return res;
    }

    /**
     * Trigger a command when the button is pressed.
     * 
     * @param cmd the command to trigger
     * @return self for chaining
     */
    public Button on_press(Command cmd) {
        this.trigger().onTrue(cmd);
        return this;
    }

    /**
     * Trigger a command when the button is released.
     * 
     * @param cmd the command to trigger
     * @return self for chaining
     */
    public Button on_release(Command cmd) {
        this.trigger().onFalse(cmd);
        return this;
    }

    /**
     * Keep executing a command when the button is held down.
     * 
     * @param cmd the command to trigger
     * @return self for chaining
     */
    public Button on_down(Command cmd) {
        this.trigger().whileTrue(cmd);
        return this;
    }

    /**
     * Keep executing a command when the button is free.
     * 
     * @param cmd the command to trigger
     * @return self for chaining
     */
    public Button on_up(Command cmd) {
        this.trigger().whileFalse(cmd);
        return this;
    }

    /**
     * Create a <code>Trigger</code> from this button for interaction with
     * <i>WPILib</i>.
     * 
     * @return new
     */
    public Trigger trigger() {
        return new Trigger(this.raw_input);
    }

}
