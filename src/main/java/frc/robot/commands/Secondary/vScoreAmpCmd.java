// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.Secondary.vLauncherSubsystem;

public class vScoreAmpCmd extends Command {
  /** Creates a new Outtake. */
  
    
    private final vLauncherSubsystem vlauncherSubsystem;
    private boolean hasNote = true;
  
  
    public vScoreAmpCmd(vLauncherSubsystem vlauncherSubsystem) {
      this.vlauncherSubsystem = vlauncherSubsystem;
      
      // Use addRequirements() here to declare subsystem dependencies.
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      hasNote = true;
    }


    // // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if(Robot.sensorOuttake.get() == true || Robot.sensorIntake.get() == true){
        LauncherRotateSubsystem.m_LauncherRotatePIDController.setReference(LauncherConstants.AmpScoreAngle,CANSparkMax.ControlType.kSmartMotion);
        vlauncherSubsystem.launcherPIDControllerTop.setReference(LauncherConstants.AmpScoreSpeed, CANSparkFlex.ControlType.kVelocity);
        if (LauncherRotateSubsystem.m_LauncherRotateEncoder.getPosition() >= LauncherConstants.AmpScoreAngle - 10){
            if(((vlauncherSubsystem.m_launcherMotorTop.getEncoder().getVelocity()) >= LauncherConstants.AmpScoreSpeed - 25) &&
                (vlauncherSubsystem.m_launcherMotorTop.getEncoder().getVelocity()) <= LauncherConstants.AmpScoreSpeed + 25) {
              IntakeSubsystem.launcherIndexerMotor.set(IntakeConstants.launcherIndexerOuttakeSpeed);
              IntakeSubsystem.indexerMotor.set(IntakeConstants.indexerOuttakeSpeed);
            }
        } 
      } else {
        hasNote = false;
      }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      IntakeSubsystem.indexerMotor.set(0);
      IntakeSubsystem.launcherIndexerMotor.set(0);
      vlauncherSubsystem.m_launcherMotorTop.set(0);
      LauncherRotateSubsystem.m_LauncherRotatePIDController.setReference(0, CANSparkMax.ControlType.kSmartMotion);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return !hasNote;
    }
  }

