// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ClimberSubsystem;


public class ClimberInitCmd extends Command {

  private final ClimberSubsystem m_climberSubsystem;
  private boolean climberInitialized = false;

  
  public ClimberInitCmd(ClimberSubsystem climberSubsystem) {
    this.m_climberSubsystem = climberSubsystem;
    //addRequirements(climberSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberInitialized = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_climberSubsystem.m_limitSwitch_R.get()){
      m_climberSubsystem.m_climberMotorR.set(.1);
    } else if(m_climberSubsystem.m_limitSwitch_R.get()) {
      m_climberSubsystem.m_climberMotorR.set(0);
    }
    
    if(!m_climberSubsystem.m_limitSwitch_L.get()){
      m_climberSubsystem.m_climberMotorL.set(.1);
    } else if(m_climberSubsystem.m_limitSwitch_L.get()){
      m_climberSubsystem.m_climberMotorL.set(0);
    }

    if (m_climberSubsystem.m_limitSwitch_R.get() &&  m_climberSubsystem.m_limitSwitch_L.get()){
      climberInitialized = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.m_climberMotorR.set(0);
    m_climberSubsystem.m_climberEncoderR.setPosition(0);
    m_climberSubsystem.m_climberMotorL.set(0);
    m_climberSubsystem.m_climberEncoderL.setPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climberInitialized;
  }
}
