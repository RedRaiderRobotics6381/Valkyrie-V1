// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ClimberSubsystem;


public class ClimbCmd extends Command {

  private final ClimberSubsystem climberSubsystem;
  private boolean climbed = false;

  public ClimbCmd(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (climberSubsystem.m_climberEncoderR.getPosition() >= -0.01 && climberSubsystem.m_climberEncoderR.getPosition() <= 12.5){
        climberSubsystem.m_climberMotorR.set(.25);
    }
    if (climberSubsystem.m_climberEncoderR.getPosition() >= 12.5) {
        climberSubsystem.m_climberMotorR.set(0);
    }

    if (climberSubsystem.m_climberEncoderL.getPosition() >= -0.01 && climberSubsystem.m_climberEncoderL.getPosition() <= 12.5){
        climberSubsystem.m_climberMotorL.set(.25);
      } 
    if (climberSubsystem.m_climberEncoderL.getPosition() >= 12.5) {
        climberSubsystem.m_climberMotorL.set(0);
    }
    if (climberSubsystem.m_climberEncoderR.getPosition() >= 12.5 && climberSubsystem.m_climberEncoderL.getPosition() >= 12.5){
      climbed = true;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     climberSubsystem.m_climberMotorL.set(0);
     climberSubsystem.m_climberMotorR.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbed;
  }
}
