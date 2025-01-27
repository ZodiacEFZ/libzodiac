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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    public void power(double ratio) {
        this.motor.set(ratio);
    }

    @Override
    public void angle(double rad) {
        this.motor.setControl(new PositionDutyCycle(rad / TALONFX_UNIT));
    }

    @Override
    public void velocity(double rads) {
        SmartDashboard.putNumber("Velocity" + this.motor.getDescription(), rads / TALONFX_UNIT);
        // Our practice suggest that `VelocityVoltage` api produces a somehow more
        // stable output than `VelocityDutyCycle`.
        this.motor.setControl(new VelocityVoltage(rads / TALONFX_UNIT));
    }

    @Override
    public void voltage(double volt) {
        this.motor.setControl(new VoltageOut(volt));
    }

    public void music(double frequency) {
        this.motor.setControl(new MusicTone(frequency).withUpdateFreqHz(50));
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

    public void setSlot0Configs(Slot0Configs slot0Configs) {
        this.motor.getConfigurator().apply(slot0Configs);
    }

    public void setMotionMagicConfigs(MotionMagicConfigs motionMagicConfigs) {
        this.motor.getConfigurator().apply(motionMagicConfigs);
    }

    public void MotionMagicPosition(double position) {
        this.motor.setControl(new MotionMagicVoltage(position / TALONFX_UNIT));
    }

    public void MotionMagicVelocity(double velocity) {
        this.motor.setControl(new MotionMagicVelocityVoltage(velocity / TALONFX_UNIT));
    }

    public void MotionMagicExpo(double position) {
        this.motor.setControl(new MotionMagicExpoVoltage(position / TALONFX_UNIT));
    }

    public void setControl(ControlRequest controlRequest) {
        this.motor.setControl(controlRequest);
    }

    public static class MusicPlayer extends Orchestra implements Sendable {
        Music music;

        public void addInstrument(TalonFXMotor motor) {
            this.addInstrument(motor.motor);
        }

        public void setInstrumentAllTracks(TalonFXMotor... motors) {
            if (this.music.track == -1) {
                this.setInstrument(motors);
                return;
            }
            this.clearInstruments();
            IntStream.range(0, motors.length).forEachOrdered(i -> this.addInstrument(motors[i], i % this.music.track));
        }

        public void setInstrument(TalonFXMotor... motors) {
            this.clearInstruments();
            for (var motor : motors) {
                this.addInstrument(motor.motor);
            }
        }

        public void addInstrument(TalonFXMotor motor, int trackNumber) {
            this.addInstrument(motor.motor, trackNumber);
        }

        public void setInstrumentAllTracks(ParentDevice... instruments) {
            this.setInstrumentAllTracks(Arrays.asList(instruments));
        }

        public void setInstrumentAllTracks(Collection<ParentDevice> instruments) {
            if (this.music.track == -1) {
                this.setInstrument(instruments);
                return;
            }
            this.clearInstruments();
            IntStream.range(0, instruments.size()).forEachOrdered(
                    i -> this.addInstrument(instruments.toArray(new ParentDevice[0])[i], i % this.music.track));
        }

        public void setInstrument(Collection<ParentDevice> instruments) {
            this.clearInstruments();
            for (var instrument : instruments) {
                this.addInstrument(instrument);
            }
        }

        public void setInstrument(ParentDevice... instruments) {
            this.clearInstruments();
            for (var instrument : instruments) {
                this.addInstrument(instrument);
            }
        }

        public void loadMusic(Music music) {
            this.music = music;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Music Player");
            builder.addBooleanProperty("Playing", this::isPlaying, this::setPlayingState);
            builder.addDoubleProperty("Time", this::getCurrentTime, null);
            builder.addDoubleProperty("Track", () -> this.music.track, null);
            builder.addStringProperty("File", () -> this.music.file, null);
        }

        public void setPlayingState(boolean playing) {
            if (playing) {
                this.play();
            } else {
                this.pause();
            }
        }

        public static class Music {
            private final String file;
            private final int track;

            Music(String file, int track) {
                this.file = file;
                this.track = track;
            }

            public String getFile() {
                return file;
            }

            public int getTrack() {
                return track;
            }
        }

        public static class Musics {
            public static final Music StarWarsMainTheme = new Music("music/StarWarsMainTheme.chrp", 1);
        }
    }
}
