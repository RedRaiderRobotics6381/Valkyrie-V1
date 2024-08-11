// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ClimberSubsystem;


public class InitCmd extends Command {

  private final ClimberSubsystem m_climberSubsystem;
  //private boolean climberInitialized = false;
  // private boolean climbed;
  // private boolean climbedL;
  // private boolean climbedR;
  private boolean climberInitialized;
  
  public InitCmd(ClimberSubsystem climberSubsystem) {
    this.m_climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);

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
  if(!m_climberSubsystem.limitSwitch_R.get()){
    m_climberSubsystem.climberMotorR.set(.25);
    //I think this can just be an else statement
  } else if(m_climberSubsystem.limitSwitch_R.get()) {
    m_climberSubsystem.climberMotorR.set(0);
  }
  
  if(!m_climberSubsystem.limitSwitch_L.get()){
    m_climberSubsystem.climberMotorL.set(.25);
    //I think this can just be an else statement
  } else if(m_climberSubsystem.limitSwitch_L.get()){
    m_climberSubsystem.climberMotorL.set(0);
  }

  if (m_climberSubsystem.limitSwitch_R.get() && m_climberSubsystem.limitSwitch_L.get()){
    climberInitialized = true;
  }
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  m_climberSubsystem.climberMotorR.set(0);
  m_climberSubsystem.climberEncoderR.setPosition(0);
  m_climberSubsystem.climberMotorL.set(0);
  m_climberSubsystem.climberEncoderL.setPosition(0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return climberInitialized;
}
}
