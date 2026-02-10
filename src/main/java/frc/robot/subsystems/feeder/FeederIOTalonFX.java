package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;         
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FeederIOTalonFX extends FeederIO {
    private final TalonFX feederMotor;
    private final CANBus feederBus;
    private TalonFXConfiguration feederMotorConfig;

    private final StatusSignal<AngularVelocity> feederVelocityRad;
    private final StatusSignal<Temperature> feederTemperature;
    private final StatusSignal<Angle> feederPosition;
    private final StatusSignal<Voltage> feederAppliedVolts;
    private final StatusSignal<Current> feederStatorCurrent;
    private final StatusSignal<Current> feederSupplyCurrent;

    public FeederIOTalonFX() {
        feederBus = new CANBus(FeederConstants.feederCANBus);
        feederMotor = new TalonFX(FeederConstants.feederMotorID, feederBus);

        feederMotorConfig = new TalonFXConfiguration();
        
        feederMotorConfig.Slot0.kP = FeederConstants.kP;
        feederMotorConfig.Slot0.kI = FeederConstants.kI;
        feederMotorConfig.Slot0.kD = FeederConstants.kD;
        feederMotorConfig.Slot0.kS = FeederConstants.kS;
        feederMotorConfig.Slot0.kV = FeederConstants.kV;

        feederMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederMotorConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.supplyCurrentLimit;

        feederMotor.getConfigurator().apply(feederMotorConfig);
        feederMotor.setNeutralMode(NeutralModeValue.Brake);

        feederVelocityRad = feederMotor.getVelocity();
        feederTemperature = feederMotor.getDeviceTemp();
        feederAppliedVolts = feederMotor.getMotorVoltage();
        feederPosition = feederMotor.getPosition();
        feederStatorCurrent = feederMotor.getStatorCurrent();
        feederSupplyCurrent = feederMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50,feederVelocityRad, 
            feederTemperature, feederAppliedVolts, feederPosition, feederStatorCurrent, feederSupplyCurrent);

        feederMotor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(feederVelocityRad, feederTemperature, 
            feederAppliedVolts, feederPosition, feederStatorCurrent, feederSupplyCurrent);

        super.position = feederPosition.getValueAsDouble();
        super.velocity = feederVelocityRad.getValueAsDouble();
        super.supplyCurrent = feederSupplyCurrent.getValueAsDouble();
        super.statorCurrent = feederStatorCurrent.getValueAsDouble();
        super.appliedVolts = feederAppliedVolts.getValueAsDouble();
        super.tempCelsius = feederTemperature.getValueAsDouble();

        DogLog.log("Feeder/Velocity", super.velocity);
        DogLog.log("Feeder/Position", super.position);
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