package frc.libzodiac.hardware;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.libzodiac.api.ZMotor;

import java.util.Arrays;
import java.util.Collection;
import java.util.stream.IntStream;

/**
 * <i>Talon FX/i> motor, such as Falcon 500 and Kraken X60/X44.
 */
public final class TalonFXMotor implements ZMotor {
    public static final double TALONFX_UNIT = 2 * Math.PI;

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
        motorOutputConfigs.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        this.motor.getConfigurator().apply(motorOutputConfigs);
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
    public void position(double rad) {
        this.motor.setControl(new PositionDutyCycle(rad / TALONFX_UNIT));
    }

    @Override
    public void velocity(double radPerSec) {
        // Our practice suggest that `VelocityVoltage` api produces a somehow more
        // stable output than `VelocityDutyCycle`.
        this.motor.setControl(new VelocityVoltage(radPerSec / TALONFX_UNIT));
    }

    @Override
    public void voltage(double volt) {
        this.motor.setControl(new VoltageOut(volt));
    }

    public void frequency(double hz) {
        this.motor.setControl(new MusicTone(hz).withUpdateFreqHz(50));
    }

    public double getPosition() {
        return this.motor.getPosition().getValue().in(Units.Radians);
    }

    public void setPosition(double rad) {
        this.motor.setPosition(rad / TALONFX_UNIT);
    }

    public double getVelocity() {
        return this.motor.getVelocity().getValue().in(Units.RadiansPerSecond);
    }

    /**
     * Attain an access point to the internal motor
     *
     * @return the motor
     */
    public TalonFX getMotor() {
        return this.motor;
    }

    public void setPid(PIDController pid) {
        this.setPid(pid.getP(), pid.getI(), pid.getD());
    }

    /**
     * Configure the PID controller arguments of this motor.
     *
     * @param kP proportional control coefficient
     * @param kI integral control coefficient
     * @param kD derivative control coefficient
     */
    public void setPid(double kP, double kI, double kD) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        this.motor.getConfigurator().apply(slot0Configs);
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

    public void MotionMagicPosition(double rad) {
        this.motor.setControl(new MotionMagicVoltage(rad / TALONFX_UNIT));
    }

    public void MotionMagicVelocity(double radPerSec) {
        this.motor.setControl(new MotionMagicVelocityVoltage(radPerSec / TALONFX_UNIT));
    }

    public void MotionMagicExpo(double rad) {
        this.motor.setControl(new MotionMagicExpoVoltage(rad / TALONFX_UNIT));
    }

    public void setControl(ControlRequest request) {
        this.motor.setControl(request);
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
            this.addInstrument(Arrays.stream(motor).map(TalonFXMotor::getMotor).toArray(ParentDevice[]::new));
        }

        public void addInstrument(ParentDevice... instruments) {
            this.instruments.addAll(Arrays.asList(instruments));
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

        public void addInstrument(Collection<ParentDevice> instruments) {
            this.instruments.addAll(instruments);
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
