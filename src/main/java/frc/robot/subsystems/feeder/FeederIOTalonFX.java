package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Motor.TalonFXWrapper;

public class FeederIOTalonFX extends FeederIO {
  private final TalonFXWrapper feederMotorWrapper;
  private final TalonFX feederMotor;
  private final CANBus feederCANBus;
  private VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
  private Debouncer feederDebouncer;

  private final StatusSignal<AngularVelocity> feederVelocityRPS;
  private final StatusSignal<Temperature> feederTemperature;
  private final StatusSignal<Voltage> feederAppliedVolts;
  private final StatusSignal<Current> feederStatorCurrent;
  private final StatusSignal<Current> feederSupplyCurrent;

  public FeederIOTalonFX() {
    feederCANBus = new CANBus(FeederConstants.feederMotor.canBus());

    feederMotorWrapper = new TalonFXWrapper(FeederConstants.feederMotor);
    feederMotor = feederMotorWrapper.getTalonFX();

    velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(super.targetRPS);

    feederDebouncer = new Debouncer(FeederConstants.fuelDebounce, DebounceType.kBoth);

    feederVelocityRPS = feederMotor.getVelocity();
    feederTemperature = feederMotor.getDeviceTemp();
    feederAppliedVolts = feederMotor.getMotorVoltage();
    feederStatorCurrent = feederMotor.getStatorCurrent();
    feederSupplyCurrent = feederMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        feederVelocityRPS,
        feederTemperature,
        feederAppliedVolts,
        feederStatorCurrent,
        feederSupplyCurrent);

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
    feederMotorWrapper.stop();
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
}

