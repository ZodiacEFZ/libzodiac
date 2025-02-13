package frc.libzodiac.hardware;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
 * <i>Talon FX/i> motor, such as Falcon 500 and Kraken X60/X44.
 */
public final class TalonFXMotor implements Motor {
    private final TalonFX motor;

    public TalonFXMotor(int can_id) {
        this.motor = new TalonFX(can_id);
    }

    public void factoryDefault() {
        this.motor.getConfigurator().apply(new TalonFXConfiguration());
        this.setAllowMusicDurDisable(true);
    }

    public void setAllowMusicDurDisable(boolean allowMusicDurDisable) {
        this.motor.getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(allowMusicDurDisable));
    }

    public void setInverted(boolean inverted) {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        this.motor.getConfigurator().refresh(motorOutputConfigs);
        this.motor.getConfigurator().apply(motorOutputConfigs.withInverted(
                inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive));
    }

    public void invert() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        this.motor.getConfigurator().refresh(motorOutputConfigs);
        motorOutputConfigs.Inverted = switch (motorOutputConfigs.Inverted) {
            case Clockwise_Positive -> InvertedValue.CounterClockwise_Positive;
            case CounterClockwise_Positive -> InvertedValue.Clockwise_Positive;
        };
        this.motor.getConfigurator().apply(motorOutputConfigs);
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

    public void frequency(Frequency frequency) {
        this.motor.setControl(new MusicTone(frequency.in(Units.Hertz)).withUpdateFreqHz(50));
    }

    public Angle getPosition() {
        return this.motor.getPosition().getValue();
    }

    public void setRelativeEncoderPosition(Angle position) {
        this.motor.setPosition(position);
    }

    public void resetRelativeEncoderPosition() {
        this.motor.setPosition(0);
    }

    public AngularVelocity getVelocity() {
        return this.motor.getVelocity().getValue();
    }

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

    public void MotionMagicPosition(Angle position) {
        this.motor.setControl(new MotionMagicVoltage(position));
    }

    public void MotionMagicPosition(Angle position, Voltage feedforward) {
        this.motor.setControl(new MotionMagicVoltage(position).withFeedForward(feedforward));
    }

    public void MotionMagicVelocity(AngularVelocity angularVelocity) {
        this.motor.setControl(new MotionMagicVelocityVoltage(angularVelocity));
    }

    public void MotionMagicVelocity(AngularVelocity angularVelocity, AngularAcceleration acceleration) {
        this.motor.setControl(new MotionMagicVelocityVoltage(angularVelocity));
    }

    public void MotionMagicVelocity(AngularVelocity angularVelocity, Voltage feedforward) {
        this.motor.setControl(new MotionMagicVelocityVoltage(angularVelocity).withFeedForward(feedforward));
    }

    public void MotionMagicVelocity(AngularVelocity angularVelocity, AngularAcceleration acceleration, Voltage feedforward) {
        this.motor.setControl(new MotionMagicVelocityVoltage(angularVelocity).withAcceleration(acceleration)
                .withFeedForward(feedforward));
    }

    public void MotionMagicExpo(Angle position) {
        this.motor.setControl(new MotionMagicExpoVoltage(position));
    }

    public void MotionMagicExpo(Angle position, Voltage feedforward) {
        this.motor.setControl(new MotionMagicExpoVoltage(position).withFeedForward(feedforward));
    }

    public void setControl(ControlRequest request) {
        this.motor.setControl(request);
    }

    public void setSensorToMechanismRatio(double ratio) {
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        this.motor.getConfigurator().refresh(feedbackConfigs);
        this.motor.getConfigurator().apply(feedbackConfigs.withSensorToMechanismRatio(ratio));
    }

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

    public static class MusicPlayer implements Sendable {
        private final Orchestra orchestra;
        private final Collection<ParentDevice> instruments;
        private String file;
        private int track;

        public MusicPlayer() {
            this.orchestra = new Orchestra();
            this.instruments = new java.util.ArrayList<>();
        }

        public void addInstrument(TalonFXMotor... motor) {
            this.instruments.addAll(Arrays.stream(motor).map((m) -> m.motor).toList());
        }

        public void addInstrument(Collection<TalonFXMotor> instruments) {
            this.instruments.addAll(instruments.stream().map((m) -> m.motor).toList());
        }

        public void clearInstruments() {
            this.instruments.clear();
        }

        public void loadMusic(String file, int track) {
            this.file = file;
            this.track = track;
            this.orchestra.loadMusic(file);
            this.setInstrumentAllTracks(this.instruments);
        }

        private void setInstrumentAllTracks(Collection<ParentDevice> instruments) {
            var instrumentsList = instruments.stream().toList();
            IntStream.range(0, instrumentsList.size())
                    .forEach(i -> this.orchestra.addInstrument(instrumentsList.get(i), i % this.track));
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Music Player");
            builder.addBooleanProperty("Playing", this::isPlaying, this::setPlayingState);
            builder.addDoubleProperty("Time", this::getCurrentTime, null);
            builder.addDoubleProperty("Track", () -> this.track, null);
            builder.addStringProperty("File", () -> this.file, null);
        }

        public boolean isPlaying() {
            return this.orchestra.isPlaying();
        }

        public void setPlayingState(boolean playing) {
            if (playing) {
                this.orchestra.play();
            } else {
                this.orchestra.pause();
            }
        }

        public double getCurrentTime() {
            return this.orchestra.getCurrentTime();
        }

        public void play() {
            this.orchestra.play();
        }

        public void pause() {
            this.orchestra.pause();
        }

        public void stop() {
            this.orchestra.stop();
        }
    }
}
