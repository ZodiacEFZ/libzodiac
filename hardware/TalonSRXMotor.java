package frc.libzodiac.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;

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
    int output = 1;

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
        output = inverted ? -1 : 1;
    }

    public void invert() {
        output = -output;
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
    public void power(double ratio) {
        this.motor.set(ControlMode.PercentOutput, output * ratio);
    }

    /**
     * @implNote Unit of the angle may not necessarily be radian but dependent on
     * sensor
     * wired to the <i>TalonSRX</i> encoder unless <code>.unit</code> is
     * properly set.
     */
    @Override
    public void angle(double rad) {
        this.motor.set(ControlMode.Position, output * rad * unit);
    }

    @Override
    public void velocity(double rads) {
        this.motor.set(ControlMode.Velocity, output * rads * unit);
    }

    @Override
    public void current(double amp) {
        this.motor.set(ControlMode.Current, output * amp);
    }

    public void setMotionMagicConfig(double kP, double kI, double kD, double kF, double cruiseVelocity, double acceleration, int sCurveStrength) {
        this.motor.config_kP(0, kP);
        this.motor.config_kI(0, kI);
        this.motor.config_kD(0, kD);
        this.motor.config_kF(0, kF);

        this.motor.configMotionCruiseVelocity(cruiseVelocity * unit);
        this.motor.configMotionAcceleration(acceleration * unit);
        this.motor.configMotionSCurveStrength(sCurveStrength);
    }

    public void MotionMagic(double rad) {
        this.motor.set(ControlMode.MotionMagic, output * rad * unit);
    }

    public void MotionMagic(double rad, double feedforward) {
        this.motor.set(ControlMode.MotionMagic, output * rad * unit, DemandType.ArbitraryFeedForward, feedforward);
    }

    public double getPosition() {
        return output * this.motor.getSelectedSensorPosition() / unit;
    }

    public void setPosition(double rad) {
        this.motor.setSelectedSensorPosition(output * rad * unit);
    }

    public TalonSRX motor() {
        return this.motor;
    }

    public void follow(TalonSRXMotor master) {
        this.motor.follow(master.motor);
    }
}
