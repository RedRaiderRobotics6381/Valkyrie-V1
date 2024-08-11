// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.LauncherConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherRotateSubsystem extends SubsystemBase {
  public CANSparkMax launcherRotateMotor;
  public SparkPIDController launcherRotatePIDController;
  public SparkAbsoluteEncoder launcherRotateEncoder;
  public static double LauncherRotateSetpoint;
  public static double RotateManualPos;
  

  /** Creates a new LauncherRotateSubsystem. 
 * @param LauncherRotateSubsystem
 * */
  public LauncherRotateSubsystem() {
        // initialize motor
        launcherRotateMotor = new CANSparkMax(LauncherConstants.kLauncherRotate, MotorType.kBrushless);

        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        launcherRotateMotor.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        launcherRotateEncoder = launcherRotateMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        launcherRotateEncoder.setPositionConversionFactor(360);
        launcherRotateEncoder.setZeroOffset(341.7); //333.9
        launcherRotateMotor.setInverted(true);
        // m_LauncherRotateEncoder.setDistancePerRotation(360);
        // m_LauncherRotateEncoder.setPositionOffset(72.5);

        launcherRotateEncoder.setInverted(true); //Maybe this is not needed, depending on the direction the arm rotates.
    
        // initialze PID controller and encoder objects
        launcherRotatePIDController = launcherRotateMotor.getPIDController();
        launcherRotatePIDController.setFeedbackDevice(launcherRotateEncoder);
        launcherRotateMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 103);
        launcherRotateMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,179);
        launcherRotateMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        launcherRotateMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        launcherRotateMotor.enableVoltageCompensation(12.0);
        launcherRotateMotor.setSmartCurrentLimit(70);
        launcherRotateMotor.setIdleMode(IdleMode.kBrake);
        launcherRotateMotor.burnFlash();  //Remove this after everything is up and running to save flash wear
    
        // set PID coefficients
        launcherRotatePIDController.setP(0.000115);
        launcherRotatePIDController.setI(0.00000000001);
        launcherRotatePIDController.setD(0.0000001);
        launcherRotatePIDController.setIZone(0.0);
        
        // This is an arbitrary feedforward value that is multiplied by the positon of the arm to account
        // for the reduction in force needed to hold the arm vertical instead of hortizontal.  The .abs
        //ensures the value is always positive.  The .cos function uses radians instead of degrees,
        // so the .toRadians converts from degrees to radians.
        launcherRotatePIDController.setFF(.005 * (Math.abs
                                        (Math.cos
                                        ((Math.toRadians(LauncherRotateSetpoint)) -
                                        (Math.toRadians(90))))));
        
        launcherRotatePIDController.setOutputRange(-1, 1); //ArmConstants.armRotatekMinOutput, ArmConstants.armRotatekMaxOutput);
    
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
        launcherRotatePIDController.setSmartMotionMaxVelocity(5000.0,0); //ArmConstants.armRotateMaxVel, ArmConstants.armRotateSmartMotionSlot);
        launcherRotatePIDController.setSmartMotionMinOutputVelocity(0.0, 0); //ArmConstants.armRotateMinVel, ArmConstants.armRotateSmartMotionSlot);
        launcherRotatePIDController.setSmartMotionMaxAccel(3000.0,0); //ArmConstants.armRotateMaxAcc, ArmConstants.armRotateSmartMotionSlot);
        launcherRotatePIDController.setSmartMotionAllowedClosedLoopError(0.01, 0); //ArmConstants.armRotateAllowedErr, ArmConstants.armRotateSmartMotionSlot);  
  }

 @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Rotator Enc Val", launcherRotateEncoder.getPosition());
        if(RobotContainer.engineerXbox.getRightY() > 0.1 || RobotContainer.engineerXbox.getRightY() < -0.1){
          launcherRotatePIDController.setReference((launcherRotateEncoder.getPosition()) +
                                                           (RobotContainer.engineerXbox.getRightY() * -20),
                                                           CANSparkMax.ControlType.kSmartMotion);                                                   
    }
  }


  
  // public Command rotateAutoPosCommand() {
  //   // implicitly require `this`
  //   return this.runOnce(() -> m_LauncherRotatePIDController.setReference(PVAim.Launcher_Pitch, CANSparkMax.ControlType.kSmartMotion));
  // }
  
  public Command rotatePosCommand(double LauncherRotateSetpoint) {
    // implicitly require `this`
    return this.runOnce(() -> launcherRotatePIDController.setReference(LauncherRotateSetpoint, CANSparkMax.ControlType.kSmartMotion));
  }

  // public Command rotateIntakeCommand() {
  //   // implicitly require `this`
  //   return this.runOnce(() -> m_LauncherRotatePIDController.setReference(127.5, CANSparkMax.ControlType.kSmartMotion));
  // }


  public void setDefaultCommand(){
    launcherRotateMotor.disable();
    //m_armPIDController.setReference(ArmRotateSetpoint, CANSparkMax.ControlType.kSmartMotion);
  }

}
  

