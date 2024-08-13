// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.Secondary.LauncherSubsystem;

public class ScoreCmd extends Command {
  /** Creates a new Outtake. */
  
    double shotAng;
    double shotAngTol;
    double shotSpd;
    double shotSpdTol;
   
    private final LauncherSubsystem m_launcherSubsystem;
    private final LauncherRotateSubsystem m_launcherRotateSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    private boolean hasNote = true;
  
  
    public ScoreCmd(double shotAng, double shotAngTol, double shotSpd, double shotSpdTol, LauncherSubsystem launcherSubsystem, LauncherRotateSubsystem launcherRotateSubsystem, IntakeSubsystem intakeSubsystem) {
      
      this.shotAng = shotAng;
      this.shotAngTol = shotAngTol;
      this.shotSpd = shotSpd;
      this.shotSpdTol = shotSpdTol;
      this.m_launcherSubsystem = launcherSubsystem;
      this.m_launcherRotateSubsystem = launcherRotateSubsystem;
      this.m_intakeSubsystem = intakeSubsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(launcherSubsystem, launcherRotateSubsystem, intakeSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      hasNote = true;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
        // // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
          if(Robot.sensorOuttake.get() == true || Robot.sensorIntake.get() == true){
            m_launcherRotateSubsystem.launcherRotatePIDController.setReference(shotAng,CANSparkMax.ControlType.kSmartMotion);
            m_launcherSubsystem.launcherPIDControllerTop.setReference(shotSpd, CANSparkFlex.ControlType.kVelocity);
            if ((Math.abs(m_launcherRotateSubsystem.launcherRotateEncoder.getPosition() -
                 shotAng) <= shotAngTol)){
                  if((Math.abs(m_launcherSubsystem.launcherMotorTop.getEncoder().getVelocity() -
                      shotSpd)) <= shotSpdTol){
                        m_intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.launcherIndexerOuttakeSpeed);
                        m_intakeSubsystem.indexerMotor.set(IntakeConstants.indexerOuttakeSpeed);
                    }
              } 
            } else {
              hasNote = false;
            }
        }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_intakeSubsystem.indexerMotor.set(0);
      m_intakeSubsystem.launcherIndexerMotor.set(0);
      //m_launcherSubsystem.launcherMotorTop.set(2000);
      m_launcherSubsystem.launcherPIDControllerTop.setReference(0, CANSparkFlex.ControlType.kVelocity);
      m_launcherRotateSubsystem.launcherRotateMotor.disable();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return !hasNote;
    }
  }

