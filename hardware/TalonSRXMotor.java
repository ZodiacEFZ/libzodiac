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
    /**
     * The motor controller.
     */
    private final TalonSRX motor;
    /**
     * The position unit of the sensor.
     */
    private AngleUnit positionUnit;
    /**
     * The ratio of the sensor to the mechanism.
     */
    private double sensorToMechanismRatio = 1;

    /**
     * Construct a new Talon SRX motor.
     *
     * @param id The CAN ID of the motor controller.
     */
    public TalonSRXMotor(int id) {
        this.motor = new TalonSRX(id);
        this.positionUnit = Units.derive(Units.Rotations).splitInto(4096).named("TalonSRXEncoderUnit").symbol("")
                .make();
    }

    /**
     * Factory default the motor controller.
     */
    public void factoryDefault() {
        this.motor.configFactoryDefault();
    }

    /**
     * Configure the slot of the motor controller.
     *
     * @param config The slot configuration.
     */
    public void setSlotConfig(SlotConfiguration config) {
        this.motor.configureSlot(config);
    }

    /**
     * Configure PID arguments of the motor.
     *
     * @param pid The PID controller.
     */
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

    /**
     * Configure the unit of the sensor.
     *
     * @param unit The unit of the sensor.
     */
    public void setUnit(AngleUnit unit) {
        this.positionUnit = unit;
    }

    /**
     * Configure the unit of the sensor.
     *
     * @param unitsPerRotation The unit of the sensor.
     */
    public void setUnit(double unitsPerRotation) {
        this.positionUnit = Units.derive(Units.Rotations).splitInto(unitsPerRotation).named("TalonSRXEncoderUnit").symbol("")
                .make();
    }

    @Override
    public void setInverted(boolean inverted) {
        this.motor.setInverted(inverted);
    }

    @Override
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

    /**
     * The position unit of the sensor.
     *
     * @return The psoition unit of the sensor.
     */
    private AngleUnit positionUnit() {
        return Units.derive(this.positionUnit).aggregate(this.sensorToMechanismRatio).named("TalonSRXEncoderUnit").symbol("")
                .make();
    }

    @Override
    public void velocity(AngularVelocity angularVelocity) {
        this.motor.set(TalonSRXControlMode.Velocity, angularVelocity.in(this.velocityUnit())); // units per 100ms
    }

    /**
     * The velocity unit of the sensor.
     *
     * @return The velocity unit of the sensor.
     */
    private AngularVelocityUnit velocityUnit() {
        TimeUnit velocityTimeUnit = Units.derive(Units.Seconds).splitInto(10).named("100ms").symbol("*100ms").make();
        return this.positionUnit().per(velocityTimeUnit);
    }

    @Override
    public void current(Current current) {
        this.motor.set(TalonSRXControlMode.Current, current.in(Units.Amps));
    }

    /**
     * Configure the motion magic of the motor.
     *
     * @param kP             The proportional gain of the PID controller.
     * @param kI             The integral gain of the PID controller.
     * @param kD             The derivative gain of the PID controller.
     * @param kF             The feedforward gain of the PID controller.
     * @param cruiseVelocity The maximum velocity of the motion magic.
     * @param acceleration   The maximum acceleration of the motion magic.
     * @param sCurveStrength The strength of the S-curve.
     */
    public void setMotionMagicConfig(double kP, double kI, double kD, double kF, AngularVelocity cruiseVelocity, AngularAcceleration acceleration, int sCurveStrength) {
        this.motor.config_kP(0, kP);
        this.motor.config_kI(0, kI);
        this.motor.config_kD(0, kD);
        this.motor.config_kF(0, kF);

        this.motor.configMotionCruiseVelocity(cruiseVelocity.in(this.velocityUnit()));
        this.motor.configMotionAcceleration(acceleration.in(this.accelerationUnit()));
        this.motor.configMotionSCurveStrength(sCurveStrength);
    }

    /**
     * The acceleration unit of the sensor.
     *
     * @return The acceleration unit of the sensor.
     */
    private AngularAccelerationUnit accelerationUnit() {
        return this.velocityUnit().per(Units.Seconds);
    }

    /*
     * Control the motor to turn to a specific position with motion magic.
     *
     * @param position The position to turn to.
     */
    public void MotionMagic(Angle position) {
        this.motor.set(TalonSRXControlMode.MotionMagic, position.in(this.positionUnit()));
    }

    /**
     * Control the motor to turn to a specific position with motion magic.
     *
     * @param position    The position to turn to.
     * @param feedforward The feedforward value.
     */
    public void MotionMagic(Angle position, double feedforward) {
        this.motor.set(TalonSRXControlMode.MotionMagic, position.in(this.positionUnit()),
                DemandType.ArbitraryFeedForward, feedforward);
    }

    /**
     * Control the motor with the specific control mode.
     *
     * @param mode  The control mode.
     * @param value The value to set.
     */
    public void set(TalonSRXControlMode mode, double value) {
        this.motor.set(mode, value);
    }

    /**
     * Control the motor with the specific control mode.
     *
     * @param mode        The control mode.
     * @param demand0     The first demand value.
     * @param demand1Type The type of the second demand value.
     * @param demand1     The second demand value.
     */
    public void set(TalonSRXControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        this.motor.set(mode, demand0, demand1Type, demand1);
    }

    /**
     * Get the position of the motor.
     */
    public Angle getPosition() {
        return this.positionUnit().of(this.motor.getSelectedSensorPosition());
    }

    /**
     * Set the position of the motor.
     *
     * @param position the position to set.
     */
    public void setRelativeEncoderPosition(Angle position) {
        this.motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.motor.setSelectedSensorPosition(position.in(this.positionUnit()));
    }

    /**
     * Reset the position of the motor.
     */
    public void resetRelativeEncoderPosition() {
        this.motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.motor.setSelectedSensorPosition(0);
    }

    /**
     * Set the sensor of the motor.
     *
     * @param sensor The sensor to set.
     */
    public void setSensor(FeedbackDevice sensor) {
        this.motor.configSelectedFeedbackSensor(sensor);
    }

    /**
     * Reset the position of the motor.
     */
    public void resetPosition() {
        this.motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.motor.setSelectedSensorPosition(0);
    }

    /**
     * Follow another Talon SRX motor.
     *
     * @param master The master motor to follow.
     */
    public void follow(TalonSRXMotor master) {
        this.follow(master, false);
    }

    /**
     * Follow another Talon SRX motor.
     *
     * @param master The master motor to follow.
     * @param invert Whether to invert the following.
     */
    public void follow(TalonSRXMotor master, boolean invert) {
        this.motor.follow(master.motor);
        this.motor.setInverted(invert ? InvertType.OpposeMaster : InvertType.FollowMaster);
    }

    /**
     * Get the velocity of the motor.
     */
    public AngularVelocity getVelocity() {
        return this.velocityUnit().of(this.motor.getSelectedSensorVelocity());
    }

    /**
     * Set the sensor phase of the motor.
     */
    public void setPhase(boolean phase) {
        this.motor.setSensorPhase(phase);
    }
}
