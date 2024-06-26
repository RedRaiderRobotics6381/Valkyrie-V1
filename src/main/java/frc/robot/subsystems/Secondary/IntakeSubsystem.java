package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    //private final I2C.Port i2cPort = I2C.Port.kOnboard;
    public CANSparkMax intakeMotor;
    public CANSparkMax indexerMotor;
    public CANSparkFlex launcherIndexerMotor;
    public static SparkPIDController intakePIDController;
    //public ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotor, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(Constants.IntakeConstants.kIndexerMotor, MotorType.kBrushless);
        launcherIndexerMotor = new CANSparkFlex(Constants.IntakeConstants.kLauncherIndexerMotor, MotorType.kBrushless);
              
        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        intakeMotor.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        
        // initialze PID controller and encoder objects
        intakeMotor.enableVoltageCompensation(12.0);
        intakeMotor.setSmartCurrentLimit(60);
        intakeMotor.setInverted(true);
        indexerMotor.setInverted(true);
        launcherIndexerMotor.setInverted(true);
        intakeMotor.burnFlash();  //Remove this after everything is up and running to save flash wear
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // int proximity = colorSensor.getProximity();
        // SmartDashboard.putNumber("Note Sensor", colorSensor.getProximity());
        // SmartDashboard.putNumber("IntakeSpeed",intakeMotor.getEncoder().getVelocity());
    }
}