// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import com.revrobotics.CANSparkFlex;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.Secondary.LauncherSubsystem;

public class ScoreAutoCmd extends Command {
  /** Creates a new Outtake. */
  
    private final LauncherSubsystem m_launcherSubsystem;
    private final LauncherRotateSubsystem m_launcherRotateSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private boolean hasNote = true;
  
  
    public ScoreAutoCmd(LauncherSubsystem launcherSubsystem, LauncherRotateSubsystem launcherRotateSubsystem, IntakeSubsystem intakeSubsystem) {
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
    @Override
    public void execute() {
      if(Robot.sensorOuttake.get() == true || Robot.sensorIntake.get() == true){
        m_launcherSubsystem.launcherPIDControllerTop.setReference(LauncherConstants.LauncherSpeedMult, CANSparkFlex.ControlType.kVelocity);
        // SmartDashboard.putNumber("Launcher Speed", m_launcherSubsystem.launcherMotorTop.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Launcher Angle",m_launcherRotateSubsystem.launcherRotateEncoder.getPosition());
        if((Math.abs(m_launcherSubsystem.launcherMotorTop.getEncoder().getVelocity() -
            LauncherConstants.LauncherSpeedMult)) <= LauncherConstants.LauncherSpeedTol){   
              m_intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.launcherIndexerOuttakeSpeed);
              m_intakeSubsystem.indexerMotor.set(IntakeConstants.indexerOuttakeSpeed); 
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
      m_launcherSubsystem.launcherPIDControllerTop.setReference(0, CANSparkFlex.ControlType.kVelocity);
      m_launcherRotateSubsystem.launcherRotateMotor.disable();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return !hasNote;
    }
  }

