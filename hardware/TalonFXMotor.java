package frc.libzodiac.hardware;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.libzodiac.api.Motor;

import java.util.Arrays;
import java.util.Collection;
import java.util.stream.IntStream;

/**
 * <i>Talon FX</i> controlled motors, such as Falcon 500 and Kraken X60/X44.
 */
public final class TalonFXMotor implements Motor {

    public static final MusicPlayer MUSIC = new MusicPlayer();

    /**
     * The motor.
     */
    private final TalonFX motor;

    /**
     * Construct a new Talon FX motor.
     *
     * @param id the CAN ID of the motor.
     */
    public TalonFXMotor(int id) {
        this.motor = new TalonFX(id);
    }

    /**
     * Factory default the motor.
     */
    public void factoryDefault() {
        this.motor.getConfigurator().apply(new TalonFXConfiguration());
        this.setAllowMusicDurDisable(true);
    }

    /**
     * Set whether the motor allows music duration disable.
     *
     * @param allowMusicDurDisable whether to allow music duration disable.
     */
    public void setAllowMusicDurDisable(boolean allowMusicDurDisable) {
        AudioConfigs audioConfigs = new AudioConfigs();
        this.motor.getConfigurator().refresh(audioConfigs);
        this.motor.getConfigurator().apply(audioConfigs.withAllowMusicDurDisable(allowMusicDurDisable));
    }

    /**
     * Set the sensor to mechanism ratio of the motor.
     *
     * @param ratio the sensor to mechanism ratio.
     */
    public void setSensorToMechanismRatio(double ratio) {
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        this.motor.getConfigurator().refresh(feedbackConfigs);
        this.motor.getConfigurator().apply(feedbackConfigs.withSensorToMechanismRatio(ratio));
    }

    /**
     * Set the motor to be continuous or not.
     * {@link TalonFXMotor#setSensorToMechanismRatio(double)} must be called before this method.
     *
     * @param continuous whether the motor is continuous or not.
     */
    public void setContinuous(boolean continuous) {
        // ContinuousWarp is the only field in ClosedLoopGeneralConfigs so we don't need to use refresh to get the current value.
        this.motor.getConfigurator().apply(new ClosedLoopGeneralConfigs().withContinuousWrap(continuous));
    }

    @Override
    public void setInverted(boolean inverted) {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        this.motor.getConfigurator().refresh(motorOutputConfigs);
        this.motor.getConfigurator().apply(motorOutputConfigs.withInverted(
                inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive));
    }

    @Override
    public void invert() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        this.motor.getConfigurator().refresh(motorOutputConfigs);
        motorOutputConfigs.Inverted = switch (motorOutputConfigs.Inverted) {
            case Clockwise_Positive -> InvertedValue.CounterClockwise_Positive;
            case CounterClockwise_Positive -> InvertedValue.Clockwise_Positive;
        };
        this.motor.getConfigurator().apply(motorOutputConfigs);
    }

    /**
     * Set the motor to brake or coast.
     *
     * @param brake whether the motor should brake.
     */
    public void setBrakeWhenNeutral(boolean brake) {
        if (brake) {
            this.motor.setNeutralMode(NeutralModeValue.Brake);
        } else {
            this.motor.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    @Override
    public void shutdown() {
        this.motor.setControl(new CoastOut());
    }

    @Override
    public void brake() {
        this.motor.setControl(new StaticBrake());
    }

    @Override
    public void power(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void position(Angle position) {
        this.motor.setControl(new PositionDutyCycle(position));
    }

    @Override
    public void velocity(AngularVelocity angularVelocity) {
        // Our practice suggest that `VelocityVoltage` api produces a somehow more
        // stable output than `VelocityDutyCycle`.
        this.motor.setControl(new VelocityVoltage(angularVelocity));
    }

    @Override
    public void voltage(Voltage voltage) {
        this.motor.setControl(new VoltageOut(voltage.in(Units.Volts)));
    }

    /**
     * Set the motor to produce a sound of a certain frequency.
     *
     * @param frequency the frequency of the sound.
     */
    public void frequency(Frequency frequency) {
        this.motor.setControl(new MusicTone(frequency.in(Units.Hertz)).withUpdateFreqHz(50));
    }

    /**
     * Get the position of the motor.
     *
     * @return the position of the motor.
     */
    public Angle getPosition() {
        return this.motor.getPosition().getValue();
    }

    /**
     * Set the position of the motor.
     *
     * @param position the position to set.
     */
    public void setRelativeEncoderPosition(Angle position) {
        this.motor.setPosition(position);
    }

    /**
     * Reset the position of the motor.
     */
    public void resetRelativeEncoderPosition() {
        this.motor.setPosition(0);
    }

    /**
     * Get the velocity of the motor.
     *
     * @return the velocity of the motor.
     */
    public AngularVelocity getVelocity() {
        return this.motor.getVelocity().getValue();
    }

    /**
     * Set the PID controller of the motor.
     *
     * @param pid the PID controller.
     */
    public void setPID(PIDController pid) {
        this.setPID(pid.getP(), pid.getI(), pid.getD());
    }

    /**
     * Configure the PID controller arguments of this motor.
     *
     * @param kP proportional control coefficient
     * @param kI integral control coefficient
     * @param kD derivative control coefficient
     */
    public void setPID(double kP, double kI, double kD) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        this.motor.getConfigurator().apply(slot0Configs);
    }

    /**
     * Control the motor to turn to a specific position with Motion Magic.
     *
     * @param position the position to turn to.
     */
    public void MotionMagicPosition(Angle position) {
        this.motor.setControl(new MotionMagicVoltage(position));
    }

    /**
     * Control the motor to turn to a specific position with Motion Magic.
     *
     * @param position    the position to turn to.
     * @param feedforward the feedforward to apply.
     */
    public void MotionMagicPosition(Angle position, Voltage feedforward) {
        this.motor.setControl(new MotionMagicVoltage(position).withFeedForward(feedforward));
    }

    /**
     * Control the motor to turn at a specific velocity with Motion Magic.
     *
     * @param angularVelocity the velocity to turn at.
     */
    public void MotionMagicVelocity(AngularVelocity angularVelocity) {
        this.motor.setControl(new MotionMagicVelocityVoltage(angularVelocity));
    }

    /**
     * Control the motor to turn at a specific velocity with Motion Magic.
     *
     * @param angularVelocity the velocity to turn at.
     * @param acceleration    the acceleration to apply.
     */
    public void MotionMagicVelocity(AngularVelocity angularVelocity, AngularAcceleration acceleration) {
        this.motor.setControl(new MotionMagicVelocityVoltage(angularVelocity));
    }

    /**
     * Control the motor to turn at a specific velocity with Motion Magic.
     *
     * @param angularVelocity the velocity to turn at.
     * @param feedforward     the feedforward to apply.
     */
    public void MotionMagicVelocity(AngularVelocity angularVelocity, Voltage feedforward) {
        this.motor.setControl(new MotionMagicVelocityVoltage(angularVelocity).withFeedForward(feedforward));
    }

    /**
     * Control the motor to turn at a specific velocity with Motion Magic.
     *
     * @param angularVelocity the velocity to turn at.
     * @param acceleration    the acceleration to apply.
     * @param feedforward     the feedforward to apply.
     */
    public void MotionMagicVelocity(AngularVelocity angularVelocity, AngularAcceleration acceleration, Voltage feedforward) {
        this.motor.setControl(new MotionMagicVelocityVoltage(angularVelocity).withAcceleration(acceleration)
                .withFeedForward(feedforward));
    }

    /**
     * Control the motor to turn to a specific position with Motion Magic Expo.
     *
     * @param position the position to turn to.
     */
    public void MotionMagicExpo(Angle position) {
        this.motor.setControl(new MotionMagicExpoVoltage(position));
    }

    /**
     * Control the motor to turn to a specific position with Motion Magic Expo.
     *
     * @param position    the position to turn to.
     * @param feedforward the feedforward to apply.
     */
    public void MotionMagicExpo(Angle position, Voltage feedforward) {
        this.motor.setControl(new MotionMagicExpoVoltage(position).withFeedForward(feedforward));
    }

    /**
     * Control the motor with the control request.
     *
     * @param request the control request.
     */
    public void setControl(ControlRequest request) {
        this.motor.setControl(request);
    }

    /**
     * Apply the configuration to the motor.
     *
     * @param configs the configuration to apply.
     */
    public void applyConfiguration(TalonFXConfiguration configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(MotorOutputConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(CurrentLimitsConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(VoltageConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(TorqueCurrentConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(FeedbackConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(DifferentialSensorsConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(DifferentialConstantsConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(OpenLoopRampsConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(ClosedLoopRampsConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(HardwareLimitSwitchConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(AudioConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(SoftwareLimitSwitchConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(MotionMagicConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(CustomParamsConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(ClosedLoopGeneralConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(Slot0Configs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(Slot1Configs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(Slot2Configs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void applyConfiguration(SlotConfigs configs) {
        this.motor.getConfigurator().apply(configs);
    }

    public void registerInstrument() {
        MUSIC.addInstrument(this);
    }

    /**
     * The music player.
     */
    public static final class MusicPlayer implements Sendable {
        /**
         * The orchestra.
         */
        private final Orchestra orchestra;
        /**
         * The instruments.
         */
        private final Collection<ParentDevice> instruments;
        /**
         * The music file to play.
         */
        private String path;
        /**
         * The track count.
         */
        private int track;

        /**
         * Construct a new music player.
         */
        public MusicPlayer() {
            this.orchestra = new Orchestra();
            this.instruments = new java.util.ArrayList<>();
        }

        /**
         * Add an instrument to the music player.
         *
         * @param motor the instrument to add.
         */
        public void addInstrument(TalonFXMotor... motor) {
            this.instruments.addAll(Arrays.stream(motor).map((m) -> m.motor).toList());
        }

        /**
         * Add an instrument to the music player.
         *
         * @param instruments the instruments to add.
         */
        public void addInstrument(Collection<TalonFXMotor> instruments) {
            this.instruments.addAll(instruments.stream().map((m) -> m.motor).toList());
        }

        /**
         * Clear all instruments.
         */
        public void clearInstruments() {
            this.instruments.clear();
        }

        /**
         * Load the music file.
         *
         * @param path  the music file to load.
         * @param track the track count.
         */
        public void loadMusic(String path, int track) {
            this.path = path;
            this.track = track;
            this.setInstrumentAllTracks(this.instruments);
            this.orchestra.loadMusic(path);
        }

        /**
         * Set the instrument to all tracks.
         *
         * @param instruments the instruments to set.
         */
        private void setInstrumentAllTracks(Collection<ParentDevice> instruments) {
            var instrumentsList = instruments.stream().toList();
            IntStream.range(0, instrumentsList.size())
                    .forEach(i -> this.orchestra.addInstrument(instrumentsList.get(i), i % this.track));
        }

        /**
         * Initialize the sendable for the music player.
         *
         * @param builder The sendable builder.
         */
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Music Player");
            builder.addBooleanProperty("Playing", this::isPlaying, this::setPlayingState);
            builder.addDoubleProperty("Time", this::getCurrentTime, null);
            builder.addDoubleProperty("Track", () -> this.track, null);
            builder.addStringProperty("File", () -> this.path, null);
        }

        /**
         * Whether the music is playing.
         *
         * @return whether the music is playing.
         */
        public boolean isPlaying() {
            return this.orchestra.isPlaying();
        }

        /**
         * Set the playing state of the music.
         *
         * @param playing whether the music is playing.
         */
        public void setPlayingState(boolean playing) {
            if (playing) {
                this.orchestra.play();
            } else {
                this.orchestra.pause();
            }
        }

        /**
         * Get the current time of the music.
         *
         * @return the current time of the music.
         */
        public double getCurrentTime() {
            return this.orchestra.getCurrentTime();
        }

        /**
         * Play the music.
         */
        public void play() {
            this.orchestra.play();
        }

        /**
         * Pause the music.
         */
        public void pause() {
            this.orchestra.pause();
        }

        /**
         * Stop the music.
         */
        public void stop() {
            this.orchestra.stop();
        }
    }
}
