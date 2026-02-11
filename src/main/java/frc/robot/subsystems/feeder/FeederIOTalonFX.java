package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;         
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class FeederIOTalonFX extends FeederIO {
    private final TalonFX feederMotor;
    private final CANBus feederBus;
    private TalonFXConfiguration feederConfig;

    private final StatusSignal<AngularVelocity> feederVelocityRad;
    private final StatusSignal<Temperature> feederTemperature;
    private final StatusSignal<Voltage> feederAppliedVolts;
    private final StatusSignal<Current> feederStatorCurrent;
    private final StatusSignal<Current> feederSupplyCurrent;

    public FeederIOTalonFX() {
        feederBus = new CANBus(FeederConstants.feederCANBus);
        feederMotor = new TalonFX(FeederConstants.feederMotorID, feederBus);

        feederConfig = new TalonFXConfiguration()
            .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(FeederConstants.maxVelocityRotsPerSec)
                .withMotionMagicAcceleration(FeederConstants.maxAccelerationRotationsPerSecSQ))
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
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive));

        PhoenixUtil.tryUntilOk(10, () -> feederMotor.getConfigurator().apply(feederConfig, 1));
        
        feederVelocityRad = feederMotor.getVelocity();
        feederTemperature = feederMotor.getDeviceTemp();
        feederAppliedVolts = feederMotor.getMotorVoltage();
        feederStatorCurrent = feederMotor.getStatorCurrent();
        feederSupplyCurrent = feederMotor.getSupplyCurrent();


        BaseStatusSignal.setUpdateFrequencyForAll(50,feederVelocityRad, 
            feederTemperature, feederAppliedVolts, feederStatorCurrent, feederSupplyCurrent);

        feederMotor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(feederVelocityRad, feederTemperature, 
            feederAppliedVolts, feederStatorCurrent, feederSupplyCurrent);

        super.velocity = feederVelocityRad.getValueAsDouble();
        super.supplyCurrent = feederSupplyCurrent.getValueAsDouble();
        super.statorCurrent = feederStatorCurrent.getValueAsDouble();
        super.appliedVolts = feederAppliedVolts.getValueAsDouble();
        super.tempCelsius = feederTemperature.getValueAsDouble();

        DogLog.log("Feeder/Velocity", super.velocity);
        DogLog.log("Feeder/SupplyCurrent", super.supplyCurrent);
        DogLog.log("Feeder/StatorCurrent", super.statorCurrent);
        DogLog.log("Feeder/AppliedVolts", super.appliedVolts);
        DogLog.log("Feeder/Temperature", super.tempCelsius);
    }

    @Override
    public void stopFeeder() {
      feederMotor.stopMotor();
    }

    @Override
    public void setFeederVoltage(double volts) {
      feederMotor.setVoltage(volts);
    }
}