// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

//import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.commands.Vision.LauncherAimCMD;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
//import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.Secondary.LauncherSubsystem;

public class ScoreAutoCmd extends Command {
  /** Creates a new Outtake. */
  
    private final LauncherSubsystem launcherSubsystem;
    private boolean hasNote = true;
  
  
    public ScoreAutoCmd(LauncherSubsystem launcherSubsystem) {
      this.launcherSubsystem = launcherSubsystem;
      
      
      // Use addRequirements() here to declare subsystem dependencies.
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
        //launcherRotateSubsystem.rotatePosCommand(LauncherConstants.posSpeaker);
        //LauncherRotateSubsystem.m_LauncherRotatePIDController.setReference(LauncherConstants.posSpeaker,CANSparkMax.ControlType.kSmartMotion);

        launcherSubsystem.m_launcherMotorTop.set(LauncherConstants.launcherMotorTopSpeed);
        System.out.println(launcherSubsystem.m_launcherMotorTop.getEncoder().getVelocity());
        if(((launcherSubsystem.m_launcherMotorTop.getEncoder().getVelocity()) >= LauncherAimCMD.LauncherSpeedMult)) {
          IntakeSubsystem.launcherIndexerMotor.set(IntakeConstants.launcherIndexerOuttakeSpeed);
          IntakeSubsystem.indexerMotor.set(IntakeConstants.indexerOuttakeSpeed);
        }
      } else {
        hasNote = false;
      }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      IntakeSubsystem.indexerMotor.set(IntakeConstants.zeroSpeed);
      IntakeSubsystem.launcherIndexerMotor.set(IntakeConstants.zeroSpeed);
      launcherSubsystem.m_launcherMotorTop.set(IntakeConstants.zeroSpeed);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return !hasNote;
    }
  }

