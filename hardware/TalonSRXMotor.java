package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import frc.libzodiac.api.Motor;

/**
 * Motors powered by <i>Talon SRX</i>, such as 775 Pro.
 */
public final class TalonSRXMotor implements Motor {
    /**
     * The velocity time unit of the sensor.
     */
    private static final TimeUnit VELOCITY_TIME_UNIT = Units.derive(Units.Seconds)
                                                            .splitInto(10)
                                                            .named("100ms")
                                                            .symbol("*100ms")
                                                            .make();
    /**
     * The motor controller.
     */
    private final TalonSRX motor;
    /**
     * Zero position of the sensor in raw unit.
     */
    public double positionZero = 0;
    /**
     * The position unit of the sensor.
     */
    private AngleUnit positionUnit = null;
    /**
     * The velocity unit of the sensor.
     */
    private AngularVelocityUnit velocityUnit = null;
    /**
     * The acceleration unit of the sensor.
     */
    private AngularAccelerationUnit accelerationUnit = null;

    /**
     * Construct a new Talon SRX motor.
     *
     * @param id The CAN ID of the motor controller.
     */
    public TalonSRXMotor(int id) {
        this.motor = new TalonSRX(id);
    }

    /**
     * Construct a new Talon SRX motor.
     *
     * @param id The CAN ID of the motor controller.
     */
    public TalonSRXMotor(int id, double zero) {
        this.motor = new TalonSRX(id);
        this.positionZero = zero;
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
     * Configure the zero position of the encoder.
     *
     * @param zero the zero position in raw units.
     */
    public void setEncoderZero(double zero) {
        this.positionZero = zero;
    }

    /**
     * Reset the encoder.
     */
    public void resetEncoder() {
        this.positionZero = 0;
        this.motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.motor.setSelectedSensorPosition(0);
    }

    public void resetAbsoluteToRelative() {
        this.setRelativeEncoderPosition(this.getPosition());
    }

    public void setEncoderContinuous(boolean continuous) {
        this.motor.configFeedbackNotContinuous(!continuous, 0);
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
    public double getPower() {
        return this.motor.getMotorOutputPercent();
    }

    @Override
    public void setPower(double percent) {
        this.motor.set(TalonSRXControlMode.PercentOutput, percent);
    }

    @Override
    public Angle getPosition() {
        return this.positionUnit().of(this.motor.getSelectedSensorPosition() - this.positionZero);
    }

    @Override
    public void setPosition(Angle position) {
        this.motor.set(TalonSRXControlMode.Position,
                       position.in(this.positionUnit()) + this.positionZero);
    }

    @Override
    public AngularVelocity getVelocity() {
        return this.velocityUnit().of(this.motor.getSelectedSensorVelocity());
    }

    @Override
    public void setVelocity(AngularVelocity angularVelocity) {
        this.motor.set(TalonSRXControlMode.Velocity,
                       angularVelocity.in(this.velocityUnit())); // units per 100ms
    }

    @Override
    public Voltage getVoltage() {
        return Units.Volts.of(this.motor.getMotorOutputVoltage());
    }

    @Override
    public Current getCurrent() {
        return Units.Amp.of(this.motor.getStatorCurrent());
    }

    @Override
    public void setCurrent(Current current) {
        this.motor.set(TalonSRXControlMode.Current, current.in(Units.Amps));
    }

    /**
     * Get the velocity unit of the motor.
     */
    private AngularVelocityUnit velocityUnit() {
        if (this.velocityUnit == null) {
            this.setUnit(4096);
        }
        return this.velocityUnit;
    }

    /**
     * Get the position unit of the motor.
     */
    private AngleUnit positionUnit() {
        if (this.positionUnit == null) {
            this.setUnit(4096);
        }
        return this.positionUnit;
    }

    /**
     * Configure the unit of the sensor.
     *
     * @param unitsPerRotation Sensor unit per rotation.
     */
    public void setUnit(double unitsPerRotation) {
        this.positionUnit = Units.derive(Units.Rotations)
                                 .splitInto(unitsPerRotation)
                                 .named("TalonSRXEncoderUnit")
                                 .symbol("")
                                 .make();
        this.velocityUnit = this.positionUnit.per(VELOCITY_TIME_UNIT);
        this.accelerationUnit = this.velocityUnit.per(Units.Seconds);
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
    public void setMotionMagicConfig(double kP, double kI, double kD, double kF,
                                     AngularVelocity cruiseVelocity,
                                     AngularAcceleration acceleration, int sCurveStrength) {
        this.motor.config_kP(0, kP);
        this.motor.config_kI(0, kI);
        this.motor.config_kD(0, kD);
        this.motor.config_kF(0, kF);

        this.motor.configMotionCruiseVelocity(cruiseVelocity.in(this.velocityUnit()));
        this.motor.configMotionAcceleration(acceleration.in(this.accelerationUnit()));
        this.motor.configMotionSCurveStrength(sCurveStrength);
    }

    /**
     * Get the acceleration unit of the motor.
     */
    private AngularAccelerationUnit accelerationUnit() {
        if (this.accelerationUnit == null) {
            this.setUnit(4096);
        }
        return this.accelerationUnit;
    }

    /*
     * Control the motor to turn to a specific position with motion magic.
     *
     * @param position The position to turn to.
     */
    public void MotionMagic(Angle position) {
        this.motor.set(TalonSRXControlMode.MotionMagic,
                       position.in(this.positionUnit()) + this.positionZero);
    }

    /**
     * Control the motor to turn to a specific position with motion magic.
     *
     * @param position    The position to turn to.
     * @param feedforward The feedforward value.
     */
    public void MotionMagic(Angle position, double feedforward) {
        this.motor.set(TalonSRXControlMode.MotionMagic,
                       position.in(this.positionUnit()) + this.positionZero,
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
    public void set(TalonSRXControlMode mode, double demand0, DemandType demand1Type,
                    double demand1) {
        this.motor.set(mode, demand0, demand1Type, demand1);
    }

    /**
     * Set the position of the motor.
     *
     * @param position the position to set.
     */
    public void setRelativeEncoderPosition(Angle position) {
        this.positionZero = 0;
        this.motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.motor.setSelectedSensorPosition(position.in(this.positionUnit()));
    }

    /**
     * Reset the position of the motor.
     */
    public void resetRelativeEncoderPosition() {
        this.positionZero = 0;
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
     * Set the sensor phase of the motor.
     */
    public void setPhase(boolean phase) {
        this.motor.setSensorPhase(phase);
    }

    /**
     * Set the neutral mode of the motor.
     *
     * @param brake Whether to brake the motor when neutral.
     */
    public void setBrakeWhenNeutral(boolean brake) {
        this.motor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Set the maximum integral accumulator of the motor.
     *
     * @param maxIntegralAccum The maximum integral accumulator.
     */
    public void setMaxIntegralAccum(int maxIntegralAccum) {
        this.motor.configMaxIntegralAccumulator(0, maxIntegralAccum);
    }

    /**
     * Set the peak output of the motor.
     *
     * @param peakOutput The peak output.
     */
    public void setPeakOutput(double peakOutput) {
        this.motor.configClosedLoopPeakOutput(0, peakOutput);
    }

    /**
     * Set the allowable error of the motor
     *
     * @param allowableError The allowable error.
     */
    public void allowableError(double allowableError) {
        this.motor.configAllowableClosedloopError(0, allowableError);
    }

    public void setSoftwareLimitSwitch(Angle reverseLimitSwitch, Angle forwardLimitSwitch) {
        this.setSoftwareLimitSwitch(reverseLimitSwitch.in(this.positionUnit()) + this.positionZero,
                                    forwardLimitSwitch.in(this.positionUnit()) + this.positionZero);
    }

    public void setSoftwareLimitSwitch(Double reverseLimitSwitch, Double forwardLimitSwitch) {
        if (reverseLimitSwitch == null) {
            this.motor.configReverseSoftLimitEnable(false);
        } else {
            this.motor.configReverseSoftLimitEnable(true);
            this.motor.configReverseSoftLimitThreshold(reverseLimitSwitch + this.positionZero);
        }
        if (forwardLimitSwitch == null) {
            this.motor.configForwardSoftLimitEnable(false);
        } else {
            this.motor.configForwardSoftLimitEnable(true);
            this.motor.configForwardSoftLimitThreshold(forwardLimitSwitch + this.positionZero);
        }
    }
}
