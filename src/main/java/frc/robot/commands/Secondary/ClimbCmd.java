// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ClimberSubsystem;


public class ClimbCmd extends Command {

  private final ClimberSubsystem m_climberSubsystem;
  private boolean climbed = false;
  //private double climbDist = 12.25;

  
  public ClimbCmd(ClimberSubsystem climberSubsystem) {
    this.m_climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);

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

    if(!m_climberSubsystem.limitSwitch_R.get()){
      m_climberSubsystem.climberMotorR.set(.75);
    } else if(m_climberSubsystem.limitSwitch_R.get()) {
      m_climberSubsystem.climberMotorR.set(0);
    }
    
    if(!m_climberSubsystem.limitSwitch_L.get()){
      m_climberSubsystem.climberMotorL.set(.75);
    } else if(m_climberSubsystem.limitSwitch_L.get()){
      m_climberSubsystem.climberMotorL.set(0);
    }

    if (m_climberSubsystem.limitSwitch_R.get() &&  m_climberSubsystem.limitSwitch_L.get()){
      climbed = true;
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.climberMotorL.set(0);
    m_climberSubsystem.climberMotorR.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbed;
  }
}
