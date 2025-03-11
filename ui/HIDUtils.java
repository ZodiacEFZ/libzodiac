package frc.libzodiac.ui;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.libzodiac.util.Base85;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.IntStream;

public class HIDUtils {
    /**
     * Represents the state of a button.
     *
     * @param raw      whether the button is currently pressed
     * @param pressed  whether the button was pressed now but not pressed previously
     * @param released whether the button was not pressed now but pressed previously
     */
    private record ButtonState(boolean raw, boolean pressed, boolean released) {
        /**
         * Constructs a ButtonState from a raw value.
         *
         * @param value The raw value of the button state.
         */
        private ButtonState(int value) {
            this((value & 0b100) != 0, (value & 0b010) != 0, (value & 0b001) != 0);
        }

        /**
         * Get the raw value of the button state as a string.
         *
         * @return The raw value of the button state as a string.
         */
        public String toString() {
            return Integer.toString(this.toInt());
        }

        /**
         * Get the raw value of the button state.
         *
         * @return The raw value of the button state.
         */
        public int toInt() {
            int rawBit = this.raw ? 1 : 0;
            int pressedBit = this.pressed ? 1 : 0;
            int releasedBit = this.released ? 1 : 0;
            return rawBit << 2 | pressedBit << 1 | releasedBit;
        }
    }

    /**
     * Represents the state of a HID device.
     *
     * @param time    the time at which the state was recorded
     * @param buttons the state of the buttons
     * @param axes    the values of the axes
     * @param pov     the angles of the POV
     */
    private record HIDState(double time, ButtonState[] buttons, double[] axes, int[] pov) {
        /**
         * Get the state as a string.
         *
         * @return The state as a string.
         */
        public String toString() {
            return time + "," + Arrays.toString(this.buttons) + "," + Arrays.toString(this.axes) +
                   "," + Arrays.toString(this.pov);
        }
    }

    /**
     * Represents a record of HID states.
     */
    public static class HIDRecord {
        private final ArrayList<HIDState> states = new ArrayList<>();

        /**
         * Constructs a HIDRecord.
         */
        private HIDRecord() {
        }

        /**
         * Constructs a HIDRecord.
         *
         * @param string The record as a string. If the string starts with "B85", it will be decoded
         *               as a Base85 string. Otherwise, it will be parsed as a raw string.
         */
        public HIDRecord(String string) {
            if (string.startsWith("B85")) {
                try {
                    string = Base85.getZ85Decoder().decode(string);
                } catch (Exception ignored) {
                    // The string is not valid Base85, so it will be parsed as a raw string
                }
            }
            String[] stateStrings = string.split("\\|");
            for (String stateString : stateStrings) {
                String[] parts = stateString.split(",");
                double time = Double.parseDouble(parts[0]);
                ButtonState[] buttons = Arrays.stream(parts[1].split(","))
                                              .mapToInt(Integer::parseInt)
                                              .mapToObj(ButtonState::new)
                                              .toArray(ButtonState[]::new);
                double[] axes = Arrays.stream(parts[2].split(","))
                                      .mapToDouble(Double::parseDouble)
                                      .toArray();
                int[] pov = Arrays.stream(parts[3].split(","))
                                  .mapToInt(Integer::parseInt)
                                  .toArray();
                this.states.add(new HIDState(time, buttons, axes, pov));
            }
        }

        /**
         * Add a state to the record.
         *
         * @param state The state to add.
         */
        private void add(HIDState state) {
            this.states.add(state);
        }

        /**
         * Clear the record.
         */
        private void clear() {
            this.states.clear();
        }

        /**
         * Get the record as a Base85 string.
         *
         * @return The record as a Base85 string.
         */
        public String toString() {
            return this.toZ85String();
        }

        /**
         * Get the record as a Base85 string.
         *
         * @return The record as a Base85 string.
         */
        public String toZ85String() {
            return "B85" + Base85.getZ85Encoder().encode(this.toRawString());
        }

        /**
         * Get the record as a raw string.
         *
         * @return The record as a raw string.
         */
        public String toRawString() {
            StringBuilder builder = new StringBuilder();
            IntStream.range(0, states.size()).forEachOrdered(i -> {
                builder.append(states.get(i).toString());
                if (i < states.size() - 1) {
                    builder.append("|");
                }
            });
            return builder.toString();
        }
    }

    /**
     * A class for recording HID states.
     */
    public static class HIDRecorder extends SubsystemBase {
        final Timer timer = new Timer();
        private final GenericHID device;
        private final HIDRecord record = new HIDRecord();
        private boolean recording = false;

        /**
         * Constructs a HIDRecorder.
         *
         * @param device The HID device to record.
         */
        public HIDRecorder(GenericHID device) {
            this.device = device;
        }

        /**
         * Start recording.
         */
        public void startRecording() {
            this.record.clear();
            this.timer.reset();
            this.timer.start();
            this.recording = true;
        }

        /**
         * Stop recording.
         *
         * @return The record.
         */
        public HIDRecord stopRecording() {
            this.recording = false;
            this.timer.stop();
            return this.record;
        }

        /**
         * Get the record.
         *
         * @return The record.
         */
        public HIDRecord getRecord() {
            return this.record;
        }

        @Override
        public void periodic() {
            if (recording) {
                ButtonState[] buttons = new ButtonState[device.getButtonCount()];
                double[] axes = new double[device.getAxisCount()];
                int[] pov = new int[device.getPOVCount()];
                Arrays.setAll(buttons, i -> new ButtonState(this.device.getRawButton(i),
                                                            this.device.getRawButtonPressed(i),
                                                            this.device.getRawButtonReleased(i)));
                Arrays.setAll(axes, this.device::getRawAxis);
                Arrays.setAll(pov, this.device::getPOV);
                this.record.add(new HIDState(this.timer.get(), buttons, axes, pov));
            }
        }
    }

    /**
     * A class for playing back HID records.
     */
    public static class HIDPlayback {
        private final Timer timer = new Timer();
        private final HIDRecord record;
        private final boolean[] presented;
        private int index = 0;
        private boolean playing = false;

        /**
         * Constructs a HIDPlayback.
         *
         * @param serializedRecord The record as a string. If the string starts with "B85", it will
         *                         be decoded as a Base85 string.
         */
        public HIDPlayback(String serializedRecord) {
            this.record = new HIDRecord(serializedRecord);
            this.presented = new boolean[this.record.states.size()];
        }

        /**
         * Constructs a HIDPlayback.
         *
         * @param record The record.
         */
        public HIDPlayback(HIDRecord record) {
            this.record = record;
            this.presented = new boolean[this.record.states.size()];
        }

        /**
         * Constructs a HIDPlayback.
         *
         * @param playback The playback to copy.
         */
        public HIDPlayback(HIDPlayback playback) {
            this.record = playback.record;
            this.presented = new boolean[this.record.states.size()];
            this.index = 0;
            this.playing = false;
        }

        /**
         * Start playing the record.
         */
        public void start() {
            this.timer.reset();
            this.timer.start();
            this.index = 0;
            this.playing = true;
        }

        /**
         * Stop playing the record.
         */
        public void reset() {
            this.timer.reset();
            this.playing = false;
        }

        /**
         * Get the button value (starting at button 1).
         *
         * <p>The buttons are returned in a single 16 bit value with one bit representing the state
         * of
         * each button. The appropriate button is returned as a boolean value.
         *
         * <p>This method returns true if the button is being held down at the time that this method
         * is
         * being called.
         *
         * @param button The button number to be read (starting at 1)
         *
         * @return The state of the button.
         */
        public boolean getRawButton(int button) {
            return this.getCurrentState().buttons[button].raw;
        }

        /**
         * Get the current state.
         *
         * @return The current state.
         */
        private HIDState getCurrentState() {
            if (this.playing) {
                double time = this.timer.get();
                while (this.presented[this.index] &&
                       this.record.states.get(this.index + 1).time < time) {
                    this.index++;
                    if (this.index >= this.record.states.size()) {
                        this.playing = false;
                        return this.record.states.get(this.record.states.size() - 1);
                    }
                }
                this.presented[this.index] = true;
                return this.record.states.get(this.index);
            }
            return new HIDState(0, new ButtonState[0], new double[0], new int[0]);
        }

        /**
         * Whether the button was pressed since the last check. Button indexes begin at 1.
         *
         * <p>This method returns true if the button went from not pressed to held down since the
         * last
         * time this method was called. This is useful if you only want to call a function once when
         * you press the button.
         *
         * @param button The button index, beginning at 1.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getRawButtonPressed(int button) {
            return this.getCurrentState().buttons[button].pressed;
        }

        /**
         * Whether the button was released since the last check. Button indexes begin at 1.
         *
         * <p>This method returns true if the button went from held down to not pressed since the
         * last
         * time this method was called. This is useful if you only want to call a function once when
         * you release the button.
         *
         * @param button The button index, beginning at 1.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getRawButtonReleased(int button) {
            return this.getCurrentState().buttons[button].released;
        }

        /**
         * Get the value of the axis.
         *
         * @param axis The axis to read, starting at 0.
         *
         * @return The value of the axis.
         */
        public double getRawAxis(int axis) {
            return this.getCurrentState().axes[axis];
        }

        /**
         * Get the angle in degrees of a POV on the HID.
         *
         * <p>The POV angles start at 0 in the up direction, and increase clockwise (e.g. right is
         * 90,
         * upper-left is 315).
         *
         * @param pov The index of the POV to read (starting at 0). Defaults to 0.
         *
         * @return the angle of the POV in degrees, or -1 if the POV is not pressed.
         */
        public int getPOV(int pov) {
            return this.getCurrentState().pov[pov];
        }

        /**
         * Get the angle in degrees of the default POV (index 0) on the HID.
         *
         * <p>The POV angles start at 0 in the up direction, and increase clockwise (e.g. right is
         * 90,
         * upper-left is 315).
         *
         * @return the angle of the POV in degrees, or -1 if the POV is not pressed.
         */
        public int getPOV() {
            return this.getCurrentState().pov[0];
        }

        /**
         * For the current HID, return the number of buttons.
         *
         * @return the number of buttons for the current HID
         */
        public int getButtonCount() {
            return this.record.states.get(0).buttons.length;
        }

        /**
         * Get the number of axes for the HID.
         *
         * @return the number of axis for the current HID
         */
        public int getAxisCount() {
            return this.record.states.get(0).axes.length;
        }

        /**
         * For the current HID, return the number of POVs.
         *
         * @return the number of POVs for the current HID
         */
        public int getPOVCount() {
            return this.record.states.get(0).pov.length;
        }
    }

    public static class XboxPlayback extends HIDPlayback {
        /**
         * Constructs a XboxPlayback.
         *
         * @param serializedRecord The record as a string. If the string starts with "B85", it will
         *                         be decoded as a Base85 string.
         */
        public XboxPlayback(String serializedRecord) {
            super(serializedRecord);
        }

        /**
         * Constructs a XboxPlayback.
         *
         * @param record The record.
         */
        public XboxPlayback(HIDRecord record) {
            super(record);
        }

        /**
         * Constructs a XboxPlayback.
         *
         * @param playback The playback to copy.
         */
        public XboxPlayback(HIDPlayback playback) {
            super(playback);
        }

        /**
         * Get the X axis value of left side of the controller. Right is positive.
         *
         * @return The axis value.
         */
        public double getLeftX() {
            return this.getRawAxis(XboxController.Axis.kLeftX.value);
        }

        /**
         * Get the X axis value of right side of the controller. Right is positive.
         *
         * @return The axis value.
         */
        public double getRightX() {
            return this.getRawAxis(XboxController.Axis.kRightX.value);
        }

        /**
         * Get the Y axis value of left side of the controller. Back is positive.
         *
         * @return The axis value.
         */
        public double getLeftY() {
            return this.getRawAxis(XboxController.Axis.kLeftY.value);
        }

        /**
         * Get the Y axis value of right side of the controller. Back is positive.
         *
         * @return The axis value.
         */
        public double getRightY() {
            return this.getRawAxis(XboxController.Axis.kRightY.value);
        }

        /**
         * Get the left trigger axis value of the controller. Note that this axis is bound to the
         * range of [0, 1] as opposed to the usual [-1, 1].
         *
         * @return The axis value.
         */
        public double getLeftTriggerAxis() {
            return this.getRawAxis(XboxController.Axis.kLeftTrigger.value);
        }

        /**
         * Get the right trigger axis value of the controller. Note that this axis is bound to the
         * range of [0, 1] as opposed to the usual [-1, 1].
         *
         * @return The axis value.
         */
        public double getRightTriggerAxis() {
            return getRawAxis(XboxController.Axis.kRightTrigger.value);
        }

        /**
         * Read the value of the A button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getAButton() {
            return getRawButton(XboxController.Button.kA.value);
        }

        /**
         * Whether the A button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getAButtonPressed() {
            return getRawButtonPressed(XboxController.Button.kA.value);
        }

        /**
         * Whether the A button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getAButtonReleased() {
            return getRawButtonReleased(XboxController.Button.kA.value);
        }

        /**
         * Read the value of the B button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getBButton() {
            return getRawButton(XboxController.Button.kB.value);
        }

        /**
         * Whether the B button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getBButtonPressed() {
            return getRawButtonPressed(XboxController.Button.kB.value);
        }

        /**
         * Whether the B button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getBButtonReleased() {
            return getRawButtonReleased(XboxController.Button.kB.value);
        }

        /**
         * Read the value of the X button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getXButton() {
            return getRawButton(XboxController.Button.kX.value);
        }

        /**
         * Whether the X button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getXButtonPressed() {
            return getRawButtonPressed(XboxController.Button.kX.value);
        }

        /**
         * Whether the X button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getXButtonReleased() {
            return getRawButtonReleased(XboxController.Button.kX.value);
        }

        /**
         * Read the value of the Y button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getYButton() {
            return getRawButton(XboxController.Button.kY.value);
        }

        /**
         * Whether the Y button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getYButtonPressed() {
            return getRawButtonPressed(XboxController.Button.kY.value);
        }

        /**
         * Whether the Y button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getYButtonReleased() {
            return getRawButtonReleased(XboxController.Button.kY.value);
        }

        /**
         * Read the value of the left bumper button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getLeftBumperButton() {
            return getRawButton(XboxController.Button.kLeftBumper.value);
        }

        /**
         * Whether the left bumper button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getLeftBumperButtonPressed() {
            return getRawButtonPressed(XboxController.Button.kLeftBumper.value);
        }

        /**
         * Whether the left bumper button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getLeftBumperButtonReleased() {
            return getRawButtonReleased(XboxController.Button.kLeftBumper.value);
        }

        /**
         * Read the value of the right bumper button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getRightBumperButton() {
            return getRawButton(XboxController.Button.kRightBumper.value);
        }

        /**
         * Whether the right bumper button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getRightBumperButtonPressed() {
            return getRawButtonPressed(XboxController.Button.kRightBumper.value);
        }

        /**
         * Whether the right bumper button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getRightBumperButtonReleased() {
            return getRawButtonReleased(XboxController.Button.kRightBumper.value);
        }

        /**
         * Read the value of the back button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getBackButton() {
            return getRawButton(XboxController.Button.kBack.value);
        }

        /**
         * Whether the back button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getBackButtonPressed() {
            return getRawButtonPressed(XboxController.Button.kBack.value);
        }

        /**
         * Whether the back button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getBackButtonReleased() {
            return getRawButtonReleased(XboxController.Button.kBack.value);
        }

        /**
         * Read the value of the start button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getStartButton() {
            return getRawButton(XboxController.Button.kStart.value);
        }

        /**
         * Whether the start button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getStartButtonPressed() {
            return getRawButtonPressed(XboxController.Button.kStart.value);
        }

        /**
         * Whether the start button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getStartButtonReleased() {
            return getRawButtonReleased(XboxController.Button.kStart.value);
        }

        /**
         * Read the value of the left stick button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getLeftStickButton() {
            return getRawButton(XboxController.Button.kLeftStick.value);
        }

        /**
         * Whether the left stick button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getLeftStickButtonPressed() {
            return getRawButtonPressed(XboxController.Button.kLeftStick.value);
        }

        /**
         * Whether the left stick button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getLeftStickButtonReleased() {
            return getRawButtonReleased(XboxController.Button.kLeftStick.value);
        }

        /**
         * Read the value of the right stick button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getRightStickButton() {
            return getRawButton(XboxController.Button.kRightStick.value);
        }

        /**
         * Whether the right stick button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getRightStickButtonPressed() {
            return getRawButtonPressed(XboxController.Button.kRightStick.value);
        }

        /**
         * Whether the right stick button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getRightStickButtonReleased() {
            return getRawButtonReleased(XboxController.Button.kRightStick.value);
        }
    }

    public static class CommandHIDPlayback {
        private final HIDPlayback playback;

        private final Map<EventLoop, Map<Integer, Trigger>> buttonCache = new HashMap<>();
        private final Map<EventLoop, Map<Pair<Integer, Double>, Trigger>> axisLessThanCache = new HashMap<>();
        private final Map<EventLoop, Map<Pair<Integer, Double>, Trigger>> axisGreaterThanCache = new HashMap<>();
        private final Map<EventLoop, Map<Pair<Integer, Double>, Trigger>> axisMagnitudeGreaterThanCache = new HashMap<>();
        private final Map<EventLoop, Map<Integer, Trigger>> povCache = new HashMap<>();

        /**
         * Constructs a CommandHIDPlayback.
         *
         * @param serializedRecord The record as a string. If the string starts with "B85", it will
         *                         be decoded as a Base85 string.
         */
        public CommandHIDPlayback(String serializedRecord) {
            this.playback = new HIDPlayback(serializedRecord);
        }

        /**
         * Constructs a CommandHIDPlayback.
         *
         * @param record The record.
         */
        public CommandHIDPlayback(HIDRecord record) {
            this.playback = new HIDPlayback(record);
        }

        /**
         * Constructs a CommandHIDPlayback.
         *
         * @param playback The playback to copy.
         */
        public CommandHIDPlayback(HIDPlayback playback) {
            this.playback = playback;
        }

        /**
         * Constructs an event instance around this button's digital signal.
         *
         * @param button the button index
         *
         * @return an event instance representing the button's digital signal attached to the
         * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         *
         * @see #button(int, EventLoop)
         */
        public Trigger button(int button) {
            return button(button, CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs an event instance around this button's digital signal.
         *
         * @param button the button index
         * @param loop   the event loop instance to attach the event to.
         *
         * @return an event instance representing the button's digital signal attached to the given
         * loop.
         */
        public Trigger button(int button, EventLoop loop) {
            var cache = buttonCache.computeIfAbsent(loop, k -> new HashMap<>());
            return cache.computeIfAbsent(button,
                                         k -> new Trigger(loop, () -> playback.getRawButton(k)));
        }

        /**
         * Constructs a Trigger instance based around the 0-degree angle (up) of the default (index
         * 0) POV on the HID, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * @return a Trigger instance based around the 0-degree angle of a POV on the HID.
         */
        public Trigger povUp() {
            return pov(0);
        }

        /**
         * Constructs a Trigger instance based around this angle of the default (index 0) POV on the
         * HID, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * <p>The POV angles start at 0 in the up direction, and increase clockwise (e.g. right is
         * 90,
         * upper-left is 315).
         *
         * @param angle POV angle in degrees, or -1 for the center / not pressed.
         *
         * @return a Trigger instance based around this angle of a POV on the HID.
         */
        public Trigger pov(int angle) {
            return pov(0, angle, CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance based around this angle of a POV on the HID.
         *
         * <p>The POV angles start at 0 in the up direction, and increase clockwise (e.g. right is
         * 90,
         * upper-left is 315).
         *
         * @param pov   index of the POV to read (starting at 0). Defaults to 0.
         * @param angle POV angle in degrees, or -1 for the center / not pressed.
         * @param loop  the event loop instance to attach the event to. Defaults to
         *              {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler
         *              button loop}.
         *
         * @return a Trigger instance based around this angle of a POV on the HID.
         */
        public Trigger pov(int pov, int angle, EventLoop loop) {
            var cache = povCache.computeIfAbsent(loop, k -> new HashMap<>());
            // angle can be -1, so use 3600 instead of 360
            return cache.computeIfAbsent(pov * 3600 + angle, k -> new Trigger(loop,
                                                                              () -> playback.getPOV(
                                                                                      pov) ==
                                                                                    angle));
        }

        /**
         * Constructs a Trigger instance based around the 45-degree angle (right up) of the default
         * (index 0) POV on the HID, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * @return a Trigger instance based around the 45-degree angle of a POV on the HID.
         */
        public Trigger povUpRight() {
            return pov(45);
        }

        /**
         * Constructs a Trigger instance based around the 90-degree angle (right) of the default
         * (index 0) POV on the HID, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * @return a Trigger instance based around the 90-degree angle of a POV on the HID.
         */
        public Trigger povRight() {
            return pov(90);
        }

        /**
         * Constructs a Trigger instance based around the 135-degree angle (right down) of the
         * default (index 0) POV on the HID, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * @return a Trigger instance based around the 135-degree angle of a POV on the HID.
         */
        public Trigger povDownRight() {
            return pov(135);
        }

        /**
         * Constructs a Trigger instance based around the 180-degree angle (down) of the default
         * (index 0) POV on the HID, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * @return a Trigger instance based around the 180-degree angle of a POV on the HID.
         */
        public Trigger povDown() {
            return pov(180);
        }

        /**
         * Constructs a Trigger instance based around the 225-degree angle (down left) of the
         * default (index 0) POV on the HID, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * @return a Trigger instance based around the 225-degree angle of a POV on the HID.
         */
        public Trigger povDownLeft() {
            return pov(225);
        }

        /**
         * Constructs a Trigger instance based around the 270-degree angle (left) of the default
         * (index 0) POV on the HID, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * @return a Trigger instance based around the 270-degree angle of a POV on the HID.
         */
        public Trigger povLeft() {
            return pov(270);
        }

        /**
         * Constructs a Trigger instance based around the 315-degree angle (left up) of the default
         * (index 0) POV on the HID, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * @return a Trigger instance based around the 315-degree angle of a POV on the HID.
         */
        public Trigger povUpLeft() {
            return pov(315);
        }

        /**
         * Constructs a Trigger instance based around the center (not pressed) position of the
         * default (index 0) POV on the HID, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * @return a Trigger instance based around the center position of a POV on the HID.
         */
        public Trigger povCenter() {
            return pov(-1);
        }

        /**
         * Constructs a Trigger instance that is true when the axis value is less than
         * {@code threshold}, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * @param axis      The axis to read, starting at 0
         * @param threshold The value below which this trigger should return true.
         *
         * @return a Trigger instance that is true when the axis value is less than the provided
         * threshold.
         */
        public Trigger axisLessThan(int axis, double threshold) {
            return axisLessThan(axis, threshold,
                                CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance that is true when the axis value is less than
         * {@code threshold}, attached to the given loop.
         *
         * @param axis      The axis to read, starting at 0
         * @param threshold The value below which this trigger should return true.
         * @param loop      the event loop instance to attach the trigger to
         *
         * @return a Trigger instance that is true when the axis value is less than the provided
         * threshold.
         */
        public Trigger axisLessThan(int axis, double threshold, EventLoop loop) {
            var cache = axisLessThanCache.computeIfAbsent(loop, k -> new HashMap<>());
            return cache.computeIfAbsent(Pair.of(axis, threshold), k -> new Trigger(loop,
                                                                                    () -> getRawAxis(
                                                                                            axis) <
                                                                                          threshold));
        }

        /**
         * Get the value of the axis.
         *
         * @param axis The axis to read, starting at 0.
         *
         * @return The value of the axis.
         */
        public double getRawAxis(int axis) {
            return playback.getRawAxis(axis);
        }

        /**
         * Constructs a Trigger instance that is true when the axis value is less than
         * {@code threshold}, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * @param axis      The axis to read, starting at 0
         * @param threshold The value above which this trigger should return true.
         *
         * @return a Trigger instance that is true when the axis value is greater than the provided
         * threshold.
         */
        public Trigger axisGreaterThan(int axis, double threshold) {
            return axisGreaterThan(axis, threshold,
                                   CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance that is true when the axis value is greater than
         * {@code threshold}, attached to the given loop.
         *
         * @param axis      The axis to read, starting at 0
         * @param threshold The value above which this trigger should return true.
         * @param loop      the event loop instance to attach the trigger to.
         *
         * @return a Trigger instance that is true when the axis value is greater than the provided
         * threshold.
         */
        public Trigger axisGreaterThan(int axis, double threshold, EventLoop loop) {
            var cache = axisGreaterThanCache.computeIfAbsent(loop, k -> new HashMap<>());
            return cache.computeIfAbsent(Pair.of(axis, threshold), k -> new Trigger(loop,
                                                                                    () -> getRawAxis(
                                                                                            axis) >
                                                                                          threshold));
        }

        /**
         * Constructs a Trigger instance that is true when the axis magnitude value is greater than
         * {@code threshold}, attached to
         * {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
         * loop}.
         *
         * @param axis      The axis to read, starting at 0
         * @param threshold The value above which this trigger should return true.
         *
         * @return a Trigger instance that is true when the deadbanded axis value is active
         * (non-zero).
         */
        public Trigger axisMagnitudeGreaterThan(int axis, double threshold) {
            return axisMagnitudeGreaterThan(axis, threshold,
                                            CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance that is true when the axis magnitude value is greater than
         * {@code threshold}, attached to the given loop.
         *
         * @param axis      The axis to read, starting at 0
         * @param threshold The value above which this trigger should return true.
         * @param loop      the event loop instance to attach the trigger to.
         *
         * @return a Trigger instance that is true when the axis magnitude value is greater than the
         * provided threshold.
         */
        public Trigger axisMagnitudeGreaterThan(int axis, double threshold, EventLoop loop) {
            var cache = axisMagnitudeGreaterThanCache.computeIfAbsent(loop, k -> new HashMap<>());
            return cache.computeIfAbsent(Pair.of(axis, threshold), k -> new Trigger(loop,
                                                                                    () -> Math.abs(
                                                                                            getRawAxis(
                                                                                                    axis)) >
                                                                                          threshold));
        }
    }

    public static class CommandXboxPlayback extends CommandHIDPlayback {
        private final XboxPlayback playback;

        /**
         * Constructs a CommandXboxPlayback.
         *
         * @param serializedRecord The record as a string. If the string starts with "B85", it will
         *                         be decoded as a Base85 string.
         */
        public CommandXboxPlayback(String serializedRecord) {
            super(serializedRecord);
            this.playback = new XboxPlayback(serializedRecord);
        }

        /**
         * Constructs a CommandXboxPlayback.
         *
         * @param record The record.
         */
        public CommandXboxPlayback(HIDRecord record) {
            super(record);
            this.playback = new XboxPlayback(record);
        }

        /**
         * Constructs a CommandXboxPlayback.
         *
         * @param playback The playback to copy.
         */
        public CommandXboxPlayback(HIDPlayback playback) {
            super(playback);
            this.playback = new XboxPlayback(playback);
        }

        /**
         * Constructs a CommandXboxPlayback.
         *
         * @param playback The playback to copy.
         */
        public CommandXboxPlayback(XboxPlayback playback) {
            super(playback);
            this.playback = playback;
        }

        /**
         * Constructs a Trigger instance around the A button's digital signal.
         *
         * @return a Trigger instance representing the A button's digital signal attached to the
         * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         *
         * @see #a(EventLoop)
         */
        public Trigger a() {
            return a(CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance around the A button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         *
         * @return a Trigger instance representing the A button's digital signal attached to the
         * given loop.
         */
        public Trigger a(EventLoop loop) {
            return button(XboxController.Button.kA.value, loop);
        }

        /**
         * Constructs a Trigger instance around the B button's digital signal.
         *
         * @return a Trigger instance representing the B button's digital signal attached to the
         * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         *
         * @see #b(EventLoop)
         */
        public Trigger b() {
            return b(CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance around the B button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         *
         * @return a Trigger instance representing the B button's digital signal attached to the
         * given loop.
         */
        public Trigger b(EventLoop loop) {
            return button(XboxController.Button.kB.value, loop);
        }

        /**
         * Constructs a Trigger instance around the X button's digital signal.
         *
         * @return a Trigger instance representing the X button's digital signal attached to the
         * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         *
         * @see #x(EventLoop)
         */
        public Trigger x() {
            return x(CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance around the X button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         *
         * @return a Trigger instance representing the X button's digital signal attached to the
         * given loop.
         */
        public Trigger x(EventLoop loop) {
            return button(XboxController.Button.kX.value, loop);
        }

        /**
         * Constructs a Trigger instance around the Y button's digital signal.
         *
         * @return a Trigger instance representing the Y button's digital signal attached to the
         * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         *
         * @see #y(EventLoop)
         */
        public Trigger y() {
            return y(CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance around the Y button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         *
         * @return a Trigger instance representing the Y button's digital signal attached to the
         * given loop.
         */
        public Trigger y(EventLoop loop) {
            return button(XboxController.Button.kY.value, loop);
        }

        /**
         * Constructs a Trigger instance around the left bumper button's digital signal.
         *
         * @return a Trigger instance representing the left bumper button's digital signal attached
         * to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         *
         * @see #leftBumper(EventLoop)
         */
        public Trigger leftBumper() {
            return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance around the left bumper button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         *
         * @return a Trigger instance representing the left bumper button's digital signal attached
         * to the given loop.
         */
        public Trigger leftBumper(EventLoop loop) {
            return button(XboxController.Button.kLeftBumper.value, loop);
        }

        /**
         * Constructs a Trigger instance around the right bumper button's digital signal.
         *
         * @return a Trigger instance representing the right bumper button's digital signal attached
         * to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         *
         * @see #rightBumper(EventLoop)
         */
        public Trigger rightBumper() {
            return rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance around the right bumper button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         *
         * @return a Trigger instance representing the right bumper button's digital signal attached
         * to the given loop.
         */
        public Trigger rightBumper(EventLoop loop) {
            return button(XboxController.Button.kRightBumper.value, loop);
        }

        /**
         * Constructs a Trigger instance around the back button's digital signal.
         *
         * @return a Trigger instance representing the back button's digital signal attached to the
         * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         *
         * @see #back(EventLoop)
         */
        public Trigger back() {
            return back(CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance around the back button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         *
         * @return a Trigger instance representing the back button's digital signal attached to the
         * given loop.
         */
        public Trigger back(EventLoop loop) {
            return button(XboxController.Button.kBack.value, loop);
        }

        /**
         * Constructs a Trigger instance around the start button's digital signal.
         *
         * @return a Trigger instance representing the start button's digital signal attached to the
         * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         *
         * @see #start(EventLoop)
         */
        public Trigger start() {
            return start(CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance around the start button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         *
         * @return a Trigger instance representing the start button's digital signal attached to the
         * given loop.
         */
        public Trigger start(EventLoop loop) {
            return button(XboxController.Button.kStart.value, loop);
        }

        /**
         * Constructs a Trigger instance around the left stick button's digital signal.
         *
         * @return a Trigger instance representing the left stick button's digital signal attached
         * to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         *
         * @see #leftStick(EventLoop)
         */
        public Trigger leftStick() {
            return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance around the left stick button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         *
         * @return a Trigger instance representing the left stick button's digital signal attached
         * to the given loop.
         */
        public Trigger leftStick(EventLoop loop) {
            return button(XboxController.Button.kLeftStick.value, loop);
        }

        /**
         * Constructs a Trigger instance around the right stick button's digital signal.
         *
         * @return a Trigger instance representing the right stick button's digital signal attached
         * to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         *
         * @see #rightStick(EventLoop)
         */
        public Trigger rightStick() {
            return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance around the right stick button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         *
         * @return a Trigger instance representing the right stick button's digital signal attached
         * to the given loop.
         */
        public Trigger rightStick(EventLoop loop) {
            return button(XboxController.Button.kRightStick.value, loop);
        }

        /**
         * Constructs a Trigger instance around the axis value of the left trigger. The returned
         * trigger will be true when the axis value is greater than 0.5.
         *
         * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5,
         * attached to the
         * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         */
        public Trigger leftTrigger() {
            return leftTrigger(0.5);
        }

        /**
         * Constructs a Trigger instance around the axis value of the left trigger. The returned
         * trigger will be true when the axis value is greater than {@code threshold}.
         *
         * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This
         *                  value should be in the range [0, 1] where 0 is the unpressed state of
         *                  the axis.
         *
         * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
         * threshold, attached to the
         * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         */
        public Trigger leftTrigger(double threshold) {
            return leftTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance around the axis value of the left trigger. The returned
         * trigger will be true when the axis value is greater than {@code threshold}.
         *
         * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This
         *                  value should be in the range [0, 1] where 0 is the unpressed state of
         *                  the axis.
         * @param loop      the event loop instance to attach the Trigger to.
         *
         * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
         * threshold, attached to the given event loop
         */
        public Trigger leftTrigger(double threshold, EventLoop loop) {
            return axisGreaterThan(XboxController.Axis.kLeftTrigger.value, threshold, loop);
        }

        /**
         * Constructs a Trigger instance around the axis value of the right trigger. The returned
         * trigger will be true when the axis value is greater than 0.5.
         *
         * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5,
         * attached to the
         * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         */
        public Trigger rightTrigger() {
            return rightTrigger(0.5);
        }

        /**
         * Constructs a Trigger instance around the axis value of the right trigger. The returned
         * trigger will be true when the axis value is greater than {@code threshold}.
         *
         * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This
         *                  value should be in the range [0, 1] where 0 is the unpressed state of
         *                  the axis.
         *
         * @return a Trigger instance that is true when the right trigger's axis exceeds the
         * provided threshold, attached to the
         * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
         */
        public Trigger rightTrigger(double threshold) {
            return rightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
        }

        /**
         * Constructs a Trigger instance around the axis value of the right trigger. The returned
         * trigger will be true when the axis value is greater than {@code threshold}.
         *
         * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This
         *                  value should be in the range [0, 1] where 0 is the unpressed state of
         *                  the axis.
         * @param loop      the event loop instance to attach the Trigger to.
         *
         * @return a Trigger instance that is true when the right trigger's axis exceeds the
         * provided threshold, attached to the given event loop
         */
        public Trigger rightTrigger(double threshold, EventLoop loop) {
            return axisGreaterThan(XboxController.Axis.kRightTrigger.value, threshold, loop);
        }

        /**
         * Get the X axis value of left side of the controller. Right is positive.
         *
         * @return The axis value.
         */
        public double getLeftX() {
            return playback.getLeftX();
        }

        /**
         * Get the X axis value of right side of the controller. Right is positive.
         *
         * @return The axis value.
         */
        public double getRightX() {
            return playback.getRightX();
        }

        /**
         * Get the Y axis value of left side of the controller. Back is positive.
         *
         * @return The axis value.
         */
        public double getLeftY() {
            return playback.getLeftY();
        }

        /**
         * Get the Y axis value of right side of the controller. Back is positive.
         *
         * @return The axis value.
         */
        public double getRightY() {
            return playback.getRightY();
        }

        /**
         * Get the left trigger axis value of the controller. Note that this axis is bound to the
         * range of [0, 1] as opposed to the usual [-1, 1].
         *
         * @return The axis value.
         */
        public double getLeftTriggerAxis() {
            return playback.getLeftTriggerAxis();
        }

        /**
         * Get the right trigger axis value of the controller. Note that this axis is bound to the
         * range of [0, 1] as opposed to the usual [-1, 1].
         *
         * @return The axis value.
         */
        public double getRightTriggerAxis() {
            return playback.getRightTriggerAxis();
        }
    }
}