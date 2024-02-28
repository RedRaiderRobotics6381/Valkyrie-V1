package frc.robot.subsystems.Secondary;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

    public CANSparkFlex m_launcherMotorTop;
    public CANSparkFlex m_launcherMotorBot;
    public static SparkPIDController launcherPIDControllerTop;
    public static SparkPIDController launcherPIDControllerBot;
    public double currentLauncherSpeed;

    double P = 0.0005;
    double I = 0.0;
    double D = 0.0;
    
    public LauncherSubsystem() {

        m_launcherMotorTop =  new CANSparkFlex(Constants.LauncherConstants.kLauncherT, MotorType.kBrushless);
        m_launcherMotorBot =  new CANSparkFlex(Constants.LauncherConstants.kLauncherB, MotorType.kBrushless);
        
        //m_encoderTop = m_launcherMotorTop.getAbsoluteEncoder();

        
        m_launcherMotorTop.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        m_launcherMotorBot.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        // // initialze PID controller and encoder objects
        m_launcherMotorBot.follow(m_launcherMotorTop, true);
        launcherPIDControllerTop = m_launcherMotorTop.getPIDController();
        // launcherPIDControllerBot = m_launcherMotorBot.getPIDController();
        //m_launcherMotorBot.setInverted(true);

        m_launcherMotorTop.enableVoltageCompensation(12.0);
        // m_launcherMotorBot.enableVoltageCompensation(12.0);
        m_launcherMotorTop.setSmartCurrentLimit(40);        
        // m_launcherMotorBot.setSmartCurrentLimit(40);
        
        m_launcherMotorTop.burnFlash();  //Remove this after everything is up and running to save flash wear
        m_launcherMotorBot.burnFlash();  //Remove this after everything is up and running to save flash wear
        
       
        launcherPIDControllerTop.setP(P);
        launcherPIDControllerTop.setI(I);
        launcherPIDControllerTop.setD(D);
        
        // launcherPIDControllerBot.setP(P);
        // launcherPIDControllerBot.setI(I);
        // launcherPIDControllerBot.setD(D);


        launcherPIDControllerTop.setSmartMotionMaxVelocity(5000.0,0); //ArmConstants.armRotateMaxVel, ArmConstants.armRotateSmartMotionSlot);
        launcherPIDControllerTop.setSmartMotionMinOutputVelocity(0.0, 0); //ArmConstants.armRotateMinVel, ArmConstants.armRotateSmartMotionSlot);
        launcherPIDControllerTop.setSmartMotionMaxAccel(1000.0,0); //ArmConstants.armRotateMaxAcc, ArmConstants.armRotateSmartMotionSlot);
        
        // launcherPIDControllerBot.setSmartMotionMaxVelocity(5000.0,0); //ArmConstants.armRotateMaxVel, ArmConstants.armRotateSmartMotionSlot);
        // launcherPIDControllerBot.setSmartMotionMinOutputVelocity(0.0, 0); //ArmConstants.armRotateMinVel, ArmConstants.armRotateSmartMotionSlot);
        // launcherPIDControllerBot.setSmartMotionMaxAccel(1000.0,0); //ArmConstants.armRotateMaxAcc, ArmConstants.armRotateSmartMotionSlot);

    }
    
    @Override
    public void periodic() {
        //currentLauncherSpeed = (m_launcherMotorTop.getAbsoluteEncoder().getVelocity()) * 60;
        //launcherPIDControllerBot.setReference(1000, CANSparkFlex.ControlType.kSmartVelocity);
    }
    
    public Command LauncherCmd(double speed) {
        // implicitly require `this`
        return this.run(() -> m_launcherMotorTop.set(.8));
    }

}