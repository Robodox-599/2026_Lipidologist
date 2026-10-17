package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class FeederIOTalonFX extends FeederIO {
  private final TalonFX feederMotor;
  private final CANBus feederCANBus;
  private final TalonFXConfiguration feederConfig;
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
  private Debouncer feederDebouncer;

  private final StatusSignal<AngularVelocity> feederVelocityRPS;
  private final StatusSignal<Temperature> feederTemperature;
  private final StatusSignal<Voltage> feederAppliedVolts;
  private final StatusSignal<Current> feederStatorCurrent;
  private final StatusSignal<Current> feederSupplyCurrent;

  public FeederIOTalonFX() {
    feederCANBus = new CANBus(FeederConstants.feederCANBus);
    feederMotor = new TalonFX(FeederConstants.feederMotorID, feederCANBus);
    velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(super.targetRPS);

    feederConfig =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(FeederConstants.kP)
                    .withKI(FeederConstants.kI)
                    .withKD(FeederConstants.kD)
                    .withKV(FeederConstants.kV)
                    .withKS(FeederConstants.kS))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(FeederConstants.statorCurrentLimitAmps)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(FeederConstants.supplyCurrentLimitAmps)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive));

    feederDebouncer = new Debouncer(FeederConstants.fuelDebounce, DebounceType.kBoth);

    feederVelocityRPS = feederMotor.getVelocity();
    feederTemperature = feederMotor.getDeviceTemp();
    feederAppliedVolts = feederMotor.getMotorVoltage();
    feederStatorCurrent = feederMotor.getStatorCurrent();
    feederSupplyCurrent = feederMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, feederVelocityRPS, feederStatorCurrent, feederSupplyCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(5, feederTemperature, feederAppliedVolts);

    PhoenixUtil.tryUntilOk(10, () -> feederMotor.getConfigurator().apply(feederConfig, 1));
    feederMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        feederVelocityRPS,
        feederTemperature,
        feederAppliedVolts,
        feederStatorCurrent,
        feederSupplyCurrent);

    super.RPS = feederVelocityRPS.getValueAsDouble();
    super.supplyCurrent = feederSupplyCurrent.getValueAsDouble();
    super.statorCurrent = feederStatorCurrent.getValueAsDouble();
    super.tempCelsius = feederTemperature.getValueAsDouble();
    super.isFuelJammed =
        feederDebouncer.calculate(
            super.statorCurrent > FeederConstants.stallingStatorCurrentAmps
                && Math.abs(super.RPS) < FeederConstants.jammedRPSTolerance);

    DogLog.log("Feeder/RPS", super.RPS);
    DogLog.log("Feeder/TargetRPS", super.targetRPS);
    DogLog.log("Feeder/SupplyCurrent", super.supplyCurrent);
    DogLog.log("Feeder/StatorCurrent", super.statorCurrent);
    DogLog.log("Feeder/Temperature", super.tempCelsius);
    DogLog.log("Feeder/isFuelJammed", isFuelJammed);
  }

  @Override
  public void stopFeeder() {
    feederMotor.setVoltage(0);
  }

  @Override
  public void setFeederVelocity(double RPS) {
    super.targetRPS = RPS;
    feederMotor.setControl(velocityTorqueCurrentFOC.withVelocity(super.targetRPS));
  }

  @Override
  public void setFeederVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }

  @Override
  public void setFeederDutyCycle(double dutyCycle) {
    feederMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
  }
}
