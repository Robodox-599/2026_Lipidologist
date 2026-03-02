package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
    private final CANBus feederCanbus;
    private final TalonFX feederMotor;
    private final TalonFXConfiguration feederConfig;

    private final VelocityVoltage m_request;

    private StatusSignal<AngularVelocity> feederVelocity;
    private StatusSignal<Current> feederStatorCurrent;
    private StatusSignal<Current> feederSupplyCurrent;
    private StatusSignal<Voltage> feederVoltage;
    private StatusSignal<Temperature> feederTemperature;

    //set certain velocity of 5000 rpm
    private FeederIOTalonFX(){
        feederCanbus = new CANBus(FeederConstants.feederCanbus);
        feederMotor = new TalonFX(FeederConstants.feederMotorID, feederCanbus);
        feederConfig = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimitEnable(true)
                                .withStatorCurrentLimit(FeederConstants.statorCurrent)
                                .withSupplyCurrentLimit(FeederConstants.supplyCurrent)

                )
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(FeederConstants.kP)
                                .withKI(FeederConstants.kI)
                                .withKD(FeederConstants.kD)
                                .withKV(FeederConstants.kV)
                                .withKS(FeederConstants.kS)
                );
        m_request = new VelocityVoltage(0);
        
        PhoenixUtil.tryUntilOk(10, () -> feederMotor.getConfigurator().apply(feederConfig, 1));


        feederVelocity = feederMotor.getVelocity();
        feederStatorCurrent = feederMotor.getStatorCurrent();
        feederSupplyCurrent = feederMotor.getSupplyCurrent();
        feederVoltage = feederMotor.getMotorVoltage();
        feederTemperature = feederMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, 
                feederVelocity, feederStatorCurrent, feederSupplyCurrent, feederVoltage, feederTemperature);
        
        feederMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(){
        BaseStatusSignal.refreshAll(feederVelocity, feederStatorCurrent, feederSupplyCurrent, feederVoltage, feederTemperature);
        super.velocity = feederVelocity.getValueAsDouble();
        super.statorCurrent = feederStatorCurrent.getValueAsDouble();
        super.supplyCurrent = feederSupplyCurrent.getValueAsDouble();
        super.voltage = feederVoltage.getValueAsDouble();
        super.temperature = feederTemperature.getValueAsDouble();

        DogLog.log("Feeder/Velocity", super.velocity);
        DogLog.log("Feeder/StatorCurrent", super.statorCurrent);
        DogLog.log("Feeder/SupplyCurrent", super.supplyCurrent);
        DogLog.log("Feeder/Voltage", super.voltage);
        DogLog.log("Feeder/Temperature", super.temperature);
    }

    @Override
    public void setVelocity(double velocity){
        feederMotor.setControl(m_request.withVelocity(velocity));
    }

    @Override
    public void stop(){
        feederMotor.setControl(m_request.withVelocity(0));
    }
    
}