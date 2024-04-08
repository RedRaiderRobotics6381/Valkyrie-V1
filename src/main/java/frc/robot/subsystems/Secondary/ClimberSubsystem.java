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

// These are the components that make up the ClimberSubsystem
public class ClimberSubsystem extends SubsystemBase{
    public static CANSparkMax climberMotorL;  
    public static CANSparkMax climberMotorR;
    public static RelativeEncoder climberEncoderR;
    public static RelativeEncoder climberEncoderL;
    public static DigitalInput limitSwitch_R;
    public static DigitalInput limitSwitch_L;

    //ShuffleboardTab tab = Shuffleboard.getTab("Climber");

    /**
    * @param ClimberCmd
    */
    public ClimberSubsystem(){
        // Declare the motors and the limit switches. ? The encoders don't need to be declaired because they are part of the motors?
      climberMotorR = new CANSparkMax(ClimberConstants.kClimberMotorR, MotorType.kBrushless);
      climberMotorL = new CANSparkMax(ClimberConstants.kClimberMotorL, MotorType.kBrushless);
      limitSwitch_R = new DigitalInput(3);
      limitSwitch_L = new DigitalInput(5);
      

      //Everything from here until @Override is resetting the defaults on the sparkmax controrllers and then entering in the
      //limitations that we want the sparkmax's to operate in.
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

      //Left Climber motor needs to be reversed
      climberMotorL.setInverted(true);

      //Setting the conversion factors for both motors
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
      // Adding the values of the motor encoders and the limit switches to the smart dashboard.
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