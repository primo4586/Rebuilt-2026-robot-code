package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimbConstants {
    // ids
    public static final int id = 0; // TODO: find Id

    // devices
    public static final TalonFX _motor = new TalonFX(id);

    // signals
    public static final StatusSignal<Voltage> voltageSignal = _motor.getMotorVoltage();
    public static final StatusSignal<Current> statorCurrentSignal = _motor.getStatorCurrent();
    public static final StatusSignal<Current> supplyCurrentSignal = _motor.getSupplyCurrent();
    public static final StatusSignal<Angle> postiton = _motor.getPosition();
    public static final StatusSignal<AngularVelocity> velocitySignal = _motor.getVelocity();
    public static final StatusSignal<AngularAcceleration> AccelerationSignal = _motor.getAcceleration();

    // requests
    public static final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);
    public static final PositionVoltage positionRequest = new PositionVoltage(0);
    public static final VelocityVoltage velocityRequest = new VelocityVoltage(0);
}
