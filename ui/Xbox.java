package frc.libzodiac.ui;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.libzodiac.Zubsystem;

/**
 * The <i>Xbox</i> game controller.
 */
public final class Xbox extends Zubsystem {

    /**
     * Internal access to api.
     */
    private final XboxController xbox;

    /**
     * Strength set for current rumble.
     */
    private double rumble_strength = 0.5;

    /**
     * Time set for current rumble.
     */
    private double rumble_time = 0;

    /**
     * Timer to determine whether rumble should end.
     */
    private final Timer rumble_timer = new Timer();

    private Xbox(int port) {
        this.xbox = new XboxController(port);
        this.rumble_timer.start();
    }

    /**
     * Bind to a controller port.
     * 
     * @param port the port on the driver station
     * @return new
     */
    public static Xbox bind(int port) {
        return new Xbox(port);
    }

    public Axis2D l() {
        return new Axis2D(this.lx(), this.ly());
    }

    /**
     * X axis of the left joystick. The output is in [-1,1].
     * 
     * @return an <code>Axis</code>
     */
    public Axis lx() {
        return Axis.with(this.xbox::getLeftX);
    }

    /**
     * Y axis of the left joystick. The output is in [-1,1].
     * 
     * @return an <code>Axis</code>
     */
    public Axis ly() {
        return Axis.with(this.xbox::getLeftY);
    }

    public Axis2D r() {
        return new Axis2D(this.rx(), this.ry());
    }

    /**
     * X axis of the right joystick. The output is in [-1,1].
     * 
     * @return an <code>Axis</code>
     */
    public Axis rx() {
        return Axis.with(this.xbox::getRightX);
    }

    /**
     * Y axis of the right joystick. The output is in [-1,1].
     * 
     * @return an <code>Axis</code>
     */
    public Axis ry() {
        return Axis.with(this.xbox::getRightY);
    }

    /**
     * The left trigger. The output is in [0,1].
     * 
     * @return an <code>Axis</code>
     */
    public Axis lt() {
        return Axis.with(this.xbox::getLeftTriggerAxis);
    }

    /**
     * The right trigger. The output is in [0,1].
     * 
     * @return an <code>Axis</code>
     */
    public Axis rt() {
        return Axis.with(this.xbox::getRightTriggerAxis);
    }

    /**
     * The left bumper.
     * 
     * @return a <code>Button</code>
     */
    public Button lb() {
        return Button.with(this.xbox::getLeftBumper);
    }

    /**
     * The right bumper.
     * 
     * @return a <code>Button</code>
     */
    public Button rb() {
        return Button.with(this.xbox::getRightBumper);
    }

    /**
     * The POV controller. The output is in [0,360] degrees.
     * 
     * @return an <code>Axis</code>
     */
    public Axis pov() {
        return Axis.with(this.xbox::getPOV);
    }

    /**
     * The POV controller that points up.
     * 
     * @return a <code>Button</code>
     */
    public Button up_pov() {
        final var pov = this.xbox.getPOV();
        return Button.with(() -> 315 <= pov || 0 <= pov && pov <= 45);
    }

    /**
     * The POV controller that points right.
     * 
     * @return a <code>Button</code>
     */
    public Button right_pov() {
        final var pov = this.xbox.getPOV();
        return Button.with(() -> 45 <= pov && pov <= 135);
    }

    /**
     * The POV controller that points down.
     * 
     * @return a <code>Button</code>
     */
    public Button down_pov() {
        final var pov = this.xbox.getPOV();
        return Button.with(() -> 135 <= pov && pov <= 225);
    }

    /**
     * The POV controller that points left.
     * 
     * @return a <code>Button</code>
     */
    public Button left_pov() {
        final var pov = this.xbox.getPOV();
        return Button.with(() -> 225 <= pov && pov <= 315);
    }

    /**
     * The A button.
     * 
     * @return a <code>Button</code>
     */
    public Button a() {
        return Button.with(this.xbox::getAButton);
    }

    /**
     * The B button.
     * 
     * @return a <code>Button</code>
     */
    public Button b() {
        return Button.with(this.xbox::getBButton);
    }

    /**
     * The X button.
     * 
     * @return a <code>Button</code>
     */
    public Button x() {
        return Button.with(this.xbox::getXButton);
    }

    /**
     * The Y button.
     * 
     * @return a <code>Button</code>
     */
    public Button y() {
        return Button.with(this.xbox::getYButton);
    }

    /**
     * Begin a controller rumble for specified time.
     * 
     * @param time the time to rumble
     */
    public void rumble(double time) {
        this.rumble(time, 0.5);
    }

    /**
     * Begin a controller rumble for specified time and strength.
     * 
     * @param time     the time to rumble
     * @param strength the strength of the rumble in (0,1)
     */
    public void rumble(double time, double strength) {
        this.rumble_timer.reset();
        this.rumble_time = time;
        this.rumble_strength = strength;
        this.rumble_timer.start();
    }

    @Override
    public Xbox update() {
        if (this.rumble_timer.get() >= this.rumble_time)
            this.set_rumble(0);
        else
            this.set_rumble(rumble_strength);
        return this;
    }

    /**
     * Set the controller rumbling strength.
     * 
     * @param strength the strength to set to, 0 for stop
     */
    public void set_rumble(double strength) {
        this.xbox.setRumble(RumbleType.kBothRumble, strength);
    }

}
