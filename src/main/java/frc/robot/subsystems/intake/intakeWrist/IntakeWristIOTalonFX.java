// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.intake.intakeWrist;

// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Temperature;
// import edu.wpi.first.units.measure.Velocity;
// import edu.wpi.first.units.measure.Voltage;
// import frc.robot.subsystems.intake.intakeWrist.IntakeWristConstants;

// /** Add your docs here. */
// public class IntakeWristIOTalonFX extends IntakeWristIO{
//     public final TalonFX intakeWristMotor;
//     public final TalonFXConfiguration intakeWristConfig;

//     public final StatusSignal<Angle> intakeWristPosition;
//     public final StatusSignal<AngularVelocity> intakeWristVelocity;
//     public final StatusSignal<Voltage> intakeWristAppliedVolts;
//     public final StatusSignal<Current> intakeWristStatorCurrent;
//     public final StatusSignal<Current> intakeWristSupplyCurrent;
//     public final StatusSignal<Temperature> intakeWristTemperature;
    
//     public IntakeWristIOTalonFX(){
//         intakeWristMotor = new TalonFX(IntakeWristConstants.intakeWristMotorID, IntakeWristConstants.intakeWristCanBus);
//         intakeWristConfig = new TalonFXConfiguration();
        
//         intakeWristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//         intakeWristConfig.CurrentLimits.StatorCurrentLimit = IntakeWristConstants.supplyCurrentLimit;
        
//         intakeWristPosition = intakeWristMotor.getPosition(); //?
//         intakeWristVelocity = intakeWristMotor.getVelocity();
//         intakeWristAppliedVolts = intakeWristMotor.getMotorVoltage();
//         intakeWristStatorCurrent = intakeWristMotor.getStatorCurrent();
//         intakeWristSupplyCurrent = intakeWristMotor.getSupplyCurrent();
//         intakeWristTemperature = intakeWristMotor.getDeviceTemp();

//         //PID

        
//     }
// }
