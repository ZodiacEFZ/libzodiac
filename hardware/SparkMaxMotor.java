package frc.libzodiac.hardware;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.libzodiac.api.Motor;

public final class SparkMaxMotor implements Motor {
    public static final AngleUnit POSITION_UNIT = Units.Rotations;
    public static final AngularVelocityUnit VELOCITY_UNIT = Units.Rotations.per(Units.Minutes);

    private final SparkMax motor;

    /**
     * Construct a new Spark Max motor.
     *
     * @param id      the CAN ID of the motor.
     * @param brushed whether the motor is brushed or brushless.
     */
    public SparkMaxMotor(int id, boolean brushed) {
        this.motor = new SparkMax(id, brushed ? SparkLowLevel.MotorType.kBrushed : SparkLowLevel.MotorType.kBrushless);
    }

    /**
     * Factory default the motor.
     */
    public void factoryDefault() {
        this.motor.configure(new SparkMaxConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.applyConfig(new SparkMaxConfig().inverted(inverted));
    }

    @Override
    public void invert() {
        this.applyConfig(new SparkMaxConfig().inverted(!this.motor.configAccessor.getInverted()));
    }

    @Override
    public void shutdown() {
        this.applyConfig(new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kCoast));
        this.motor.stopMotor();
    }

    @Override
    public void brake() {
        this.applyConfig(new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake));
        this.motor.stopMotor();
    }

    @Override
    public void power(double percent) {
        this.motor.getClosedLoopController().setReference(percent, SparkBase.ControlType.kDutyCycle);
    }

    @Override
    public void position(Angle position) {
        this.motor.getClosedLoopController().setReference(position.in(POSITION_UNIT), SparkBase.ControlType.kPosition);
    }

    @Override
    public void velocity(AngularVelocity angularVelocity) {
        this.motor.getClosedLoopController()
                  .setReference(angularVelocity.in(VELOCITY_UNIT), SparkBase.ControlType.kVelocity);
    }

    @Override
    public void voltage(Voltage voltage) {
        this.motor.setVoltage(voltage.in(Units.Volts));
    }

    @Override
    public void current(Current current) {
        this.motor.getClosedLoopController().setReference(current.in(Units.Amps), SparkBase.ControlType.kCurrent);
    }

    public void MAXMotionPosition(Angle position) {
        this.motor.getClosedLoopController()
                  .setReference(position.in(POSITION_UNIT), SparkBase.ControlType.kMAXMotionPositionControl);
    }

    public void MAXMotionVelocity(AngularVelocity angularVelocity) {
        this.motor.getClosedLoopController()
                  .setReference(angularVelocity.in(VELOCITY_UNIT), SparkBase.ControlType.kMAXMotionVelocityControl);
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
        var config = new SparkMaxConfig();
        config.closedLoop.pid(kP, kI, kD);
        this.applyConfig(config);
    }

    /**
     * Append specified config to the motor. This does not reset all configurations.
     *
     * @param config the motor config
     */
    public void applyConfig(SparkBaseConfig config) {
        this.motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

}
