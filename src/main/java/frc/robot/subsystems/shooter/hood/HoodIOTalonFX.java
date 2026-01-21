package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX extends HoodIO {
    //motors + configuration
    private final TalonFX hoodMotor;
    TalonFXConfiguration hoodConfiguration;

    //motion magic
    private MotionMagicVoltage motionMagic;

    //status signals
    private final StatusSignal<AngularVelocity> hoodVelocityRad;
    private final StatusSignal<Temperature> hoodTemperature;
    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<Voltage> hoodAppliedVolts;
    private final StatusSignal<Current> hoodStatorCurrent;
    private final StatusSignal<Current> hoodSupplyCurrent;

    public HoodIOTalonFX() {
        //motors + configuration
        hoodMotor = new TalonFX(HoodConstants.hoodMotorID, HoodConstants.hoodCANBus);
        hoodConfiguration = new TalonFXConfiguration();

        //applying PID to configuration
        hoodConfiguration.Slot0.kP = HoodConstants.hoodRealkP;
        hoodConfiguration.Slot0.kI = HoodConstants.hoodRealkI;
        hoodConfiguration.Slot0.kD = HoodConstants.hoodRealkD;
        hoodConfiguration.Slot0.kG = HoodConstants.hoodRealkG;
        hoodConfiguration.Slot0.kV = HoodConstants.hoodRealkV;
        hoodConfiguration.Slot0.kS = HoodConstants.hoodRealkS;

        //current limits
        hoodConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfiguration.CurrentLimits.SupplyCurrentLimit = HoodConstants.supplyCurrentLimit;

        //other configuration stuff
        hoodMotor.setNeutralMode(NeutralModeValue.Brake);

        //applying configuration
        PhoenixUtil.tryUntilOk(10, () -> hoodMotor.getConfigurator().apply(hoodConfiguration, 1));

        //status signals
        hoodVelocityRad = hoodMotor.getVelocity();
        hoodTemperature = hoodMotor.getDeviceTemp();
        hoodPosition = hoodMotor.getPosition();
        hoodAppliedVolts = hoodMotor.getMotorVoltage();
        hoodStatorCurrent = hoodMotor.getStatorCurrent();
        hoodSupplyCurrent = hoodMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, hoodVelocityRad, hoodTemperature, 
        hoodPosition, hoodAppliedVolts, hoodStatorCurrent, hoodSupplyCurrent);

        hoodMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(){
        BaseStatusSignal.refreshAll(hoodVelocityRad, hoodTemperature, 
        hoodPosition, hoodAppliedVolts, hoodStatorCurrent, hoodSupplyCurrent);

        super.position = hoodPosition.getValueAsDouble();
        super.velocity = hoodVelocityRad.getValueAsDouble();
        super.isHoodInPosition = 
            Math.abs(super.position - super.wantedPosition) < HoodConstants.positionTolerance;

        DogLog.log("Hood/Position", super.position);
        DogLog.log("Hood/Velocity", super.velocity);
        DogLog.log("Hood/TargetPosition", super.wantedPosition);
        DogLog.log("Hood/isHoodAtPosition", super.isHoodInPosition);
    }

    @Override
    public void setPosition(double position) {
        wantedPosition = MathUtil.clamp(position, HoodConstants.hoodMinAngle, HoodConstants.hoodMaxAngle);

        super.wantedPosition = wantedPosition;

        motionMagic = new MotionMagicVoltage(position).withSlot(0).withEnableFOC(true);
        
        hoodMotor.setControl(motionMagic);
    }

    @Override
    public void setVoltage(double voltage) {
        hoodMotor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        hoodMotor.stopMotor();
    }
}
