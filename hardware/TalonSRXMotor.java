package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.libzodiac.api.Motor;

/**
 * Motors powered by <i>Talon SRX</i>, such as 775 Pro.
 */
public final class TalonSRXMotor implements Motor {
    private final TalonSRX motor;
    private AngleUnit positionUnit;
    private double sensorToMechanismRatio = 1;

    public TalonSRXMotor(int can_id) {
        this.motor = new TalonSRX(can_id);
        this.positionUnit = Units.derive(Units.Rotations).splitInto(4096).named("TalonSRXEncoderUnit").symbol("")
                .make();
    }

    public void factoryDefault() {
        this.motor.configFactoryDefault();
    }

    public void setSlotConfig(SlotConfiguration config) {
        this.motor.configureSlot(config);
    }

    public void setPID(PIDController pid) {
        this.setPID(pid.getP(), pid.getI(), pid.getD());
    }

    /**
     * Configure PID arguments of the motor.
     *
     * @param kP PID proportional term factor
     * @param kI PID integral term factor
     * @param kD PID derivative term factor
     */
    public void setPID(double kP, double kI, double kD) {
        this.motor.config_kP(0, kP);
        this.motor.config_kI(0, kI);
        this.motor.config_kD(0, kD);
    }

    /**
     * @param ratio The ratio of the sensor to the mechanism.
     */
    public void setSensorToMechanismRatio(double ratio) {
        this.sensorToMechanismRatio = ratio;
    }

    public void setUnit(AngleUnit unit) {
        this.positionUnit = unit;
    }

    /**
     * Configure the sensor unit on the encoder
     *
     * @param unitsPerRad how many units one radian is equivalent to
     */
    public void setUnit(double unitsPerRad) {
        this.positionUnit = Units.derive(Units.Radians).splitInto(unitsPerRad).named("TalonSRXEncoderUnit").symbol("")
                .make();
    }

    public void setInverted(boolean inverted) {
        this.motor.setInverted(inverted);
    }

    public void invert() {
        this.motor.setInverted(!this.motor.getInverted());
    }

    @Override
    public void shutdown() {
        this.motor.setNeutralMode(NeutralMode.Coast);
        this.motor.neutralOutput();
    }

    @Override
    public void brake() {
        this.motor.setNeutralMode(NeutralMode.Brake);
        this.motor.neutralOutput();
    }

    @Override
    public void power(double percent) {
        this.motor.set(TalonSRXControlMode.PercentOutput, percent);
    }

    @Override
    public void position(Angle position) {
        this.motor.set(TalonSRXControlMode.Position, position.in(this.positionUnit()));
    }

    private AngleUnit positionUnit() {
        return Units.derive(this.positionUnit).aggregate(sensorToMechanismRatio).named("TalonSRXEncoderUnit").symbol("")
                .make();
    }

    @Override
    public void velocity(AngularVelocity angularVelocity) {
        this.motor.set(TalonSRXControlMode.Velocity, angularVelocity.in(this.velocityUnit())); // units per 100ms
    }

    private AngularVelocityUnit velocityUnit() {
        TimeUnit velocityTimeUnit = Units.derive(Units.Seconds).splitInto(10).named("100ms").symbol("*100ms").make();
        return this.positionUnit().per(velocityTimeUnit);
    }

    @Override
    public void current(Current current) {
        this.motor.set(TalonSRXControlMode.Current, current.in(Units.Amps));
    }

    public void setMotionMagicConfig(double kP, double kI, double kD, double kF, double cruiseVelocity,
                                     double acceleration, int sCurveStrength) {
        this.motor.config_kP(0, kP);
        this.motor.config_kI(0, kI);
        this.motor.config_kD(0, kD);
        this.motor.config_kF(0, kF);

        this.motor.configMotionCruiseVelocity(cruiseVelocity * this.unit);
        this.motor.configMotionAcceleration(acceleration * this.unit);
        this.motor.configMotionSCurveStrength(sCurveStrength);
    }

    public void MotionMagic(double rad) {
        this.motor.set(TalonSRXControlMode.MotionMagic, rad * this.unit);
    }

    public void MotionMagic(double rad, double feedforward) {
        this.motor.set(TalonSRXControlMode.MotionMagic, rad * this.unit, DemandType.ArbitraryFeedForward, feedforward);
    }

    public void set(TalonSRXControlMode mode, double value) {
        this.motor.set(mode, value);
    }

    public void set(TalonSRXControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        this.motor.set(mode, demand0, demand1Type, demand1);
    }

    public Angle getPosition() {
        return this.positionUnit().of(this.motor.getSelectedSensorPosition());
    }

    public void setRelativeEncoderPosition(Angle position) {
        this.motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.motor.setSelectedSensorPosition(position.in(this.positionUnit()));
    }

    public void resetRelativeEncoderPosition() {
        this.motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.motor.setSelectedSensorPosition(0);
    }

    public void setSensor(FeedbackDevice sensor) {
        this.motor.configSelectedFeedbackSensor(sensor);
    }

    public void resetPosition() {
        this.motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.motor.setSelectedSensorPosition(0);
    }

    public void follow(TalonSRXMotor master) {
        this.follow(master, true);
    }

    public void follow(TalonSRXMotor master, boolean invert) {
        this.motor.follow(master.motor);
        this.motor.setInverted(invert ? InvertType.OpposeMaster : InvertType.FollowMaster);
    }

    public AngularVelocity getVelocity() {
        return this.velocityUnit().of(this.motor.getSelectedSensorVelocity());
    }

    public void setPhase(boolean phase) {
        this.motor.setSensorPhase(phase);
    }
}
