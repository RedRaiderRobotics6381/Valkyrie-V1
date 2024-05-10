package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
// import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
// import com.revrobotics.ColorSensorV3.ColorSensorResolution;
// import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.LEDCurrent;
import com.revrobotics.ColorSensorV3.LEDPulseFrequency;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {

    public I2C.Port i2cPort = I2C.Port.kOnboard;
    public CANSparkMax intakeMotor;
    public CANSparkMax indexerMotor;
    public CANSparkFlex launcherIndexerMotor;
    public static SparkPIDController intakePIDController;
    public static ColorSensorV3 colorSensor;
    public Color detectedColor;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotor, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(Constants.IntakeConstants.kIndexerMotor, MotorType.kBrushless);
        launcherIndexerMotor = new CANSparkFlex(Constants.IntakeConstants.kLauncherIndexerMotor, MotorType.kBrushless);
        colorSensor = new ColorSensorV3(i2cPort);
        detectedColor = colorSensor.getColor();
        // colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit,
        //                            ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain6x);
        colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes8bit,
                                       ProximitySensorMeasurementRate.kProxRate12ms);
        colorSensor.configureProximitySensorLED(LEDPulseFrequency.kFreq100kHz, LEDCurrent.kPulse25mA, 100);

              
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
        //int proximity = Robot.colorSensorIntake.getProximity();
        //colorSensor.getProximity();
        //colorSensor.isConnected();
        SmartDashboard.putNumber("Note Sensor", colorSensor.getProximity());
        // SmartDashboard.putNumber("IntakeSpeed",intakeMotor.getEncoder().getVelocity());
    }
}