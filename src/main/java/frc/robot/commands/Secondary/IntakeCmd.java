// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LEDsSubSystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;

public class IntakeCmd extends Command {

  
  private boolean hasNote = false;

  public IntakeCmd() {
    
  
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasNote = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.sensorOuttake.get() == true){
      hasNote = true;
    } else {
      LauncherRotateSubsystem.m_LauncherRotatePIDController.setReference(LauncherConstants.posIntake,CANSparkMax.ControlType.kSmartMotion);
      IntakeSubsystem.indexerMotor.set(IntakeConstants.indexerIntakeSpeed);
      IntakeSubsystem.intakeMotor.set(IntakeConstants.intakeSpeed);
      IntakeSubsystem.launcherIndexerMotor.set(IntakeConstants.launcherIndexerSpeed);
      //System.out.println(Robot.sensorIntake.get());

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LEDsSubSystem.setLED(.71);
    IntakeSubsystem.indexerMotor.set(IntakeConstants.zeroSpeed);
    IntakeSubsystem.intakeMotor.set(IntakeConstants.zeroSpeed);
    IntakeSubsystem.launcherIndexerMotor.set(IntakeConstants.zeroSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasNote;
  }
}
