package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import frc.libzodiac.api.ZMotor;

/**
 * Motors powered by <i>Talon SRX</i>, such as 775 Pro.
 */
public final class TalonSRXMotor implements ZMotor {
    private final TalonSRX motor;
    /**
     * Unit of the sensor wired to the <i>TalonSRX</i> encoder, represents how many
     * radians a raw sensor unit is equal to.
     */
    public double unit = 4096 / (2 * Math.PI);

    public TalonSRXMotor(int can_id) {
        this.motor = new TalonSRX(can_id);
    }

    public void factoryDefault() {
        this.motor.configFactoryDefault();
    }

    public void setPid(PIDController pid) {
        this.setPid(pid.getP(), pid.getI(), pid.getD());
    }

    /**
     * Configure PID arguments of the motor.
     *
     * @param kP PID proportional term factor
     * @param kI PID integral term factor
     * @param kD PID derivative term factor
     */
    public void setPid(double kP, double kI, double kD) {
        this.motor.config_kP(0, kP);
        this.motor.config_kI(0, kI);
        this.motor.config_kD(0, kD);
    }

    /**
     * Configure the sensor unit on the encoder to make methods like
     * <code>.angle(double)</code> work properly under radians.
     *
     * @param unit how many radians a sensor raw unit is equal to
     */
    public void setUnit(double unit) {
        this.unit = unit;
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

    /**
     * @implNote Unit of the angle may not necessarily be radian but dependent on
     * sensor
     * wired to the <i>TalonSRX</i> encoder unless <code>.unit</code> is
     * properly set.
     */
    @Override
    public void position(double rad) {
        this.motor.set(TalonSRXControlMode.Position, rad * this.unit);
    }

    @Override
    public void velocity(double radPerSec) {
        this.motor.set(TalonSRXControlMode.Velocity, radPerSec * this.unit);
    }

    @Override
    public void current(double amp) {
        this.motor.set(TalonSRXControlMode.Current, amp);
    }

    public void setMotionMagicConfig(double kP, double kI, double kD, double kF, double cruiseVelocity, double acceleration, int sCurveStrength) {
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

    public double getPosition() {
        return this.motor.getSelectedSensorPosition() / this.unit;
    }

    public void setPosition(double rad) {
        this.motor.setSelectedSensorPosition(rad * this.unit);
    }

    public void setSensor(FeedbackDevice sensor) {
        this.motor.configSelectedFeedbackSensor(sensor);
    }

    public void resetPosition() {
        this.motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.motor.setSelectedSensorPosition(0);
    }

    public TalonSRX motor() {
        return this.motor;
    }

    public void follow(TalonSRXMotor master) {
        this.follow(master, true);
    }

    public void follow(TalonSRXMotor master, boolean invert) {
        this.motor.follow(master.motor);
        this.motor.setInverted(invert ? InvertType.OpposeMaster : InvertType.FollowMaster);
    }

    public double getVelocity() {
        return this.motor.getSelectedSensorVelocity() / this.unit;
    }
}
