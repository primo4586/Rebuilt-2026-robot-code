

public class ShooterConstants {

    public static final int MOTOR_ID = 23;
    public static final int SENSOR_ID = 9;
  
    public static final double GEARING =
        2.0; // TODO: find value i mean it dosn't really matter but still.
  
    public static final SparkMax motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    public static final DigitalInput opticalSensor = new DigitalInput(SENSOR_ID);

    public static final DoubleSupplier voltageSupplier =
        () -> motor.getBusVoltage() * motor.getAppliedOutput();
    public static final DoubleSupplier currentSupplier = () -> motor.getOutputCurrent();
    public static final BooleanSupplier opticalSensorSupplier = () -> opticalSensor.get();
  
    public static final double SHOOT_VOLTAGE = 8.4;
    public static final double SHOOT_TIME = 0.8;

  
    // configs
    public static final int MAX_CURRENT = 80;
    public static final IdleMode IDLE_MODE = IdleMode.kCoast;
    public static final boolean INVERTED = true;
    public static final SparkMaxConfig configs = new SparkMaxConfig();
  
    public static class ShooterSimConstants extends ShooterConstants {
      public static final DCMotor MOTOR_TYPE = DCMotor.getNeo550(1);
      public static final SparkMaxSim motorSim = new SparkMaxSim(motor, MOTOR_TYPE);
      public static final double OPTICAL_SENSOR_TIME = 2;
      public static final FlywheelSim sim =
          new FlywheelSim(LinearSystemId.createFlywheelSystem(MOTOR_TYPE, 0.01, GEARING), MOTOR_TYPE);
    }
}
