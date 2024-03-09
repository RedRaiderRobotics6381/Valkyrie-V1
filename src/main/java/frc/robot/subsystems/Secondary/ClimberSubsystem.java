package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

//is this working?


public class ClimberSubsystem extends SubsystemBase{
    public CANSparkMax m_climberMotorL;  
    public CANSparkMax m_climberMotorR;
    public RelativeEncoder m_climberEncoderR;
    public RelativeEncoder m_climberEncoderL;

    /**
    * @param ClimberCmd
    */
    public ClimberSubsystem(){
        // Declare the motors
      m_climberMotorR = new CANSparkMax(ClimberConstants.kClimberMotorR, MotorType.kBrushless);
      m_climberMotorL = new CANSparkMax(ClimberConstants.kClimberMotorL, MotorType.kBrushless);

      /**
       * The RestoreFactoryDefaults method can be used to reset the configuration parameters
       * in the SPARK MAX to their factory default state. If no argument is passed, these
       * parameters will not persist between power cycles
       */
      m_climberMotorR.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
      m_climberMotorL.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below        
      
      m_climberEncoderR = m_climberMotorR.getEncoder();
      m_climberEncoderL = m_climberMotorL.getEncoder();

      m_climberMotorL.setInverted(true);

      m_climberEncoderR.setPositionConversionFactor(.179); 
      m_climberEncoderL.setPositionConversionFactor(.179);

      m_climberMotorR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_climberMotorR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 14);
      m_climberMotorR.enableVoltageCompensation(12.0);
      m_climberMotorR.setSmartCurrentLimit(40);
      m_climberMotorR.setIdleMode(IdleMode.kBrake);

      m_climberMotorL.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_climberMotorL.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 14);
      m_climberMotorL.enableVoltageCompensation(12.0);
      m_climberMotorL.setSmartCurrentLimit(40);
      m_climberMotorL.setIdleMode(IdleMode.kBrake);

      m_climberMotorR.burnFlash(); //Remove this after everything is up and running to save flash wear
      m_climberMotorL.burnFlash(); //Remove this after everything is up and running to save flash wear

    }

  @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("RClimber Enc Val", m_climberEncoderR.getPosition());
      SmartDashboard.putNumber("LClimber Enc Val", m_climberEncoderL.getPosition());
    }
}