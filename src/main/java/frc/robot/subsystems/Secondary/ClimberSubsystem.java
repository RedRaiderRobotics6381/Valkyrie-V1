package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
//import frc.robot.commands.Secondary.ClimberInitCmd;

//is this working?


public class ClimberSubsystem extends SubsystemBase{
    public CANSparkMax climberMotorL;  
    public CANSparkMax climberMotorR;
    public RelativeEncoder climberEncoderR;
    public RelativeEncoder climberEncoderL;
    public DigitalInput limitSwitch_R;
    public DigitalInput limitSwitch_L;
    //ShuffleboardTab tab = Shuffleboard.getTab("Climber");

    /**
    * @param ClimberCmd
    */
    public ClimberSubsystem(){
        // Declare the motors
      climberMotorR = new CANSparkMax(ClimberConstants.kClimberMotorR, MotorType.kBrushless);
      climberMotorL = new CANSparkMax(ClimberConstants.kClimberMotorL, MotorType.kBrushless);
      limitSwitch_R = new DigitalInput(3);
      limitSwitch_L = new DigitalInput(5);
      


      /**
       * The RestoreFactoryDefaults method can be used to reset the configuration parameters
       * in the SPARK MAX to their factory default state. If no argument is passed, these
       * parameters will not persist between power cycles
       */
      climberMotorR.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
      climberMotorL.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below        
      
      climberEncoderR = climberMotorR.getEncoder();
      climberEncoderL = climberMotorL.getEncoder();
      // if (m_limitSwitch_R.get()) {
      //   m_climberEncoderR.setPosition(0);
      // }
      // if (m_limitSwitch_L.get()) {
      //   m_climberEncoderL.setPosition(0);
      // }

      climberMotorL.setInverted(true);

      climberEncoderR.setPositionConversionFactor(.179); 
      climberEncoderL.setPositionConversionFactor(.179);

      //m_climberMotorR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      //m_climberMotorR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
      climberMotorR.enableVoltageCompensation(12.0);
      climberMotorR.setSmartCurrentLimit(40);
      climberMotorR.setIdleMode(IdleMode.kBrake);

      //m_climberMotorL.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      //m_climberMotorL.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
      climberMotorL.enableVoltageCompensation(12.0);
      climberMotorL.setSmartCurrentLimit(40);
      climberMotorL.setIdleMode(IdleMode.kBrake);

      climberMotorR.burnFlash(); // Remove this after everything is up and running to save flash wear
      climberMotorL.burnFlash(); //Remove this after everything is up and running to save flash wear

    }

  @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("RClimber Enc Val", climberEncoderR.getPosition());
      SmartDashboard.putNumber("LClimber5Enc Val", climberEncoderL.getPosition());
      SmartDashboard.putBoolean("Right Limit Switch", limitSwitch_R.get());
      SmartDashboard.putBoolean("Left Limit Switch", limitSwitch_L.get());
    }
  

    
  //   public Command climberInitCmdL() {
  //   // implicitly require `this`
  //   return this.run(() -> ClimberInitCmd());
  // }
}