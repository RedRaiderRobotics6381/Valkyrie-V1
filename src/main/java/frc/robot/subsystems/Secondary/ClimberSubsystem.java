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
// =======
//     public static RelativeEncoder m_climberEncoder;

    // public static ProfiledPIDController m_climberPIDController;
    //public static Encoder ClimberEncoder;
    // private static double kS = 0.0;
    // private static double kG = 0.0;
    // private static double kV = 0.0;
    // private static double kP = 0.0;
    // private static double kI = 0.0;
    // private static double kD = 0.0;
    // private static double kDt  = 0.0;
    // private static double kMaxVelocity = 1000;
    // private static double kMaxAcceleration = 500;
    // private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    // public static ElevatorFeedforward m_climberFF;
    

    /**
    * @param ClimberCmd
    */
    public ClimberSubsystem(){
        // Declare the motors

        //ClimberEncoder = new Encoder(1, 2);
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

        // m_climberMotorL.follow(m_climberMotorR);
        //m_climberPIDController = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);

        // initialze PID controller and encoder objects

        // m_climberPIDController = m_climberMotorRight.getPIDController();
        // m_climberPIDController.setFeedbackDevice(ClimberEncoder);
        
        // m_climberMotorR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_climberMotorR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        // m_climberMotorR.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0.25);
        m_climberMotorR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 14);
        m_climberMotorR.enableVoltageCompensation(12.0);
        m_climberMotorR.setSmartCurrentLimit(40);
        m_climberMotorR.setIdleMode(IdleMode.kBrake);

        // m_climberMotorL.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_climberMotorL.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        // m_climberMotorL.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0.25);
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
      // if (RobotContainer.engineerXbox.getRawButton(2) == true){
      //   if (m_climberEncoderR.getPosition() >= 0 && m_climberEncoderR.getPosition() <= 13){
      //     m_climberMotorR.set(.25);}
      //   if (m_climberEncoderL.getPosition() >= 0 && m_climberEncoderL.getPosition() <= 13){
      //     m_climberMotorL.set(.25);}
      // } else if (RobotContainer.engineerXbox.getRawButton(3) == true){
      //   if (m_climberEncoderR.getPosition() >= 0 && m_climberEncoderR.getPosition() <= 13){
      //     m_climberMotorR.set(-.25);}
      //   if (m_climberEncoderL.getPosition() >= 0 && m_climberEncoderL.getPosition() <= 13){
      //     m_climberMotorL.set(-.25);}
      // } else{
      //   m_climberMotorL.set(0);
      //   m_climberMotorR.set(0);
      // }

    }

    // public Boolean climbCmd(){
    //     boolean climbed = false;
    //     if (m_climberEncoderR.getPosition() >= -0.01 && m_climberEncoderR.getPosition() <= 10){
    //       m_climberMotorR.set(-.25);
    //     } else {
    //       m_climberMotorR.set(0);
    //     }

    //     if (m_climberEncoderL.getPosition() >= -0.01 && m_climberEncoderL.getPosition() <= 10){
    //       m_climberMotorL.set(-.25);
    //     } else {
    //       m_climberMotorL.set(0);
    //     }
    //     if (m_climberEncoderR.getPosition() == 10 && m_climberEncoderL.getPosition() == 10){
    //       return climbed = true;
    //     }
    //     return climbed;
    // }
    
    // public Boolean lowerCmd(){
    //     boolean lowered = false;
    //     if (m_climberEncoderR.getPosition() >= -0.01 && m_climberEncoderR.getPosition() <= 10){
    //       m_climberMotorR.set(.25);
    //     } else {
    //       m_climberMotorL.set(0);
    //     }

    //     if (m_climberEncoderL.getPosition() >= -0.01 && m_climberEncoderL.getPosition() <= 10){
    //       m_climberMotorL.set(.25);
    //     } else {
    //       m_climberMotorL.set(0);
    //     }
    //     if (m_climberEncoderR.getPosition() == -0.01 && m_climberEncoderL.getPosition() == -0.01){
    //       return lowered = true;
    //     }
    //     return lowered;
    // }
}