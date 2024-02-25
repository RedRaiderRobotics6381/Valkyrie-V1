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
    public static SparkPIDController launcherPIDController;
    
    public LauncherSubsystem() {

        m_launcherMotorTop =  new CANSparkFlex(Constants.LauncherConstants.kLauncherT, MotorType.kBrushless);
        m_launcherMotorBot =  new CANSparkFlex(Constants.LauncherConstants.kLauncherB, MotorType.kBrushless);

        m_launcherMotorBot.follow(m_launcherMotorTop, true);
        // m_launcherMotorTop.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        // // initialze PID controller and encoder objects
        // launcherPIDController = m_launcherMotorTop.getPIDController();
        // m_launcherMotorTop.enableVoltageCompensation(12.0);
        // m_launcherMotorTop.setSmartCurrentLimit(40);
        // m_launcherMotorTop.burnFlash();  //Remove this after everything is up and running to save flash wear
        
        // launcherPIDController.setOutputRange(0, 1.0); //ArmConstants.armRotatekMinOutput, ArmConstants.armRotatekMaxOutput);
        
        // launcherPIDController.setP(0.0);
        // launcherPIDController.setI(0.0);
        // launcherPIDController.setD(0.0);

        // launcherPIDController.setSmartMotionMaxVelocity(5000.0,0); //ArmConstants.armRotateMaxVel, ArmConstants.armRotateSmartMotionSlot);
        // launcherPIDController.setSmartMotionMinOutputVelocity(0.0, 0); //ArmConstants.armRotateMinVel, ArmConstants.armRotateSmartMotionSlot);
        // launcherPIDController.setSmartMotionMaxAccel(10000.0,0); //ArmConstants.armRotateMaxAcc, ArmConstants.armRotateSmartMotionSlot);
    }
    
    @Override
    public void periodic() {

    }
    
    public Command LauncherCmd(double speed) {
        // implicitly require `this`
        return this.run(() -> m_launcherMotorTop.set(.8));
    }

}