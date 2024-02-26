package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    public static CANSparkMax intakeMotor;
    public static CANSparkMax indexerMotor;
    public static CANSparkFlex launcherIndexerMotor;
    public static SparkPIDController intakePIDController;
    //LauncherRotateSubsystem launcherRotateSubsystem = new LauncherRotateSubsystem();

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotor, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(Constants.IntakeConstants.kIndexerMotor, MotorType.kBrushless);
        launcherIndexerMotor = new CANSparkFlex(Constants.IntakeConstants.kLauncherIndexerMotor, MotorType.kBrushless);

        
        // indexerMotor.follow(intakeMotor);
        // launcherIndexerMotor.follow(intakeMotor);

                
        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        intakeMotor.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        
        // initialze PID controller and encoder objects
        // intakePIDController = intakeMotor.getPIDController();
        intakeMotor.enableVoltageCompensation(12.0);
        intakeMotor.setSmartCurrentLimit(60);
        intakeMotor.setInverted(true);
        indexerMotor.setInverted(true);
        launcherIndexerMotor.setInverted(true);
        intakeMotor.burnFlash();  //Remove this after everything is up and running to save flash wear
        
        
        //intakePIDController.setOutputRange(0, 1.0); //ArmConstants.armRotatekMinOutput, ArmConstants.armRotatekMaxOutput);
        
        // intakePIDController.setP(0.0);
        // intakePIDController.setI(0.0);
        // intakePIDController.setD(0.0);

        /**
         * Smart Motion coefficients are set on a SparkMaxPIDController object
         * 
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
         * the pid controller in Smart Motion mode
         * - setSmartMotionMinOutputVelocity() will put a lower bound in
         * RPM of the pid controller in Smart Motion mode
         * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
         * of the pid controller in Smart Motion mode
         * - setSmartMotionAllowedClosedLoopError() will set the max allowed
         * error for the pid controller in Smart Motion mode
         */
        // intakePIDController.setSmartMotionMaxVelocity(5000.0,0); //ArmConstants.armRotateMaxVel, ArmConstants.armRotateSmartMotionSlot);
        // intakePIDController.setSmartMotionMinOutputVelocity(0.0, 0); //ArmConstants.armRotateMinVel, ArmConstants.armRotateSmartMotionSlot);
        // intakePIDController.setSmartMotionMaxAccel(3000.0,0); //ArmConstants.armRotateMaxAcc, ArmConstants.armRotateSmartMotionSlot);
        //intakePIDController.setSmartMotionAllowedClosedLoopError(0.01, 0); //ArmConstants.armRotateAllowedErr, ArmConstants.armRotateSmartMotionSlot);  
    }

    @Override
  public void periodic() {


}
    
    // public Command InitialIntakeCmd() {
    //     // implicitly require `this`
    //     return this.run(() -> {
    //         //intakePIDController.setReference(2500, CANSparkMax.ControlType.kSmartVelocity);
    //         //LauncherRotateSubsystem.rotateIntakeCommand();
    //         intakeMotor.set(-1);
    //         indexerMotor.set(-.1);
    //         launcherIndexerMotor.set(-.1);
    //         if (Robot.sensorIntake.get() == true){
    //             //intakePIDController.setReference(0, CANSparkMax.ControlType.kSmartVelocity);
    //             intakeMotor.set(.0);
                
    //             //LEDs.setLED(.65);
    //         }
    //     });
    // }

    // public Command LaunchIntakeCmd() {
    //     // implicitly require `this`
    //     return this.runOnce(() -> {
    //         //intakePIDController.setReference(2500, CANSparkMax.ControlType.kSmartVelocity);
    //         //LauncherRotateSubsystem.rotateIntakeCommand();
    //         //intakeMotor.set(-1);
    //         //indexerMotor.set(-.1);
    //         launcherIndexerMotor.set(-1);
    //         // if (Robot.sensorIntake.get() == true){
    //         //     //intakePIDController.setReference(0, CANSparkMax.ControlType.kSmartVelocity);
    //         //     intakeMotor.set(.0);
    //         //     LEDs.setLED(.65);
    //         // }
    //     });
    // }

    //     public Command LaunchCmd(double speed) {
    //         // implicitly require `this`
    //         return this.run(() -> {
    //             //intakePIDController.setReference(speed, CANSparkMax.ControlType.kSmartVelocity);
    //             intakeMotor.set(.5);

    //         });
    //     //set to final speed once tested
        
    // }

    // public Command IntakeReverseCmd() {
    //     // implicitly require `this`
    //     return this.runOnce(() -> intakePIDController.setReference(-2500, CANSparkMax.ControlType.kSmartVelocity));
    //     //set to final speed once tested
        
    // }

}