// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ClimberSubsystem;

public class LowerCmd extends Command {

  private final ClimberSubsystem m_climberSubsystem;
  private boolean lowered = false;
  private double lowerDist = 13.0;

  public LowerCmd(ClimberSubsystem climberSubsystem) {
    this.m_climberSubsystem = climberSubsystem;
    //addRequirements(climberSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lowered = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_climberSubsystem.m_climberEncoderR.getPosition()) <= lowerDist){
      m_climberSubsystem.m_climberMotorR.set(-.75);
    } else{
      m_climberSubsystem.m_climberMotorR.set(0);
    }

    if (Math.abs(m_climberSubsystem.m_climberEncoderL.getPosition()) <= lowerDist){
      m_climberSubsystem.m_climberMotorL.set(-.75);
    } else{
      m_climberSubsystem.m_climberMotorL.set(0);
    }
  
    if (Math.abs(m_climberSubsystem.m_climberEncoderR.getPosition()) <= lowerDist && Math.abs(m_climberSubsystem.m_climberEncoderL.getPosition()) <= lowerDist){
    lowered = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.m_climberMotorL.set(0);
    m_climberSubsystem.m_climberMotorR.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lowered;
  }
}
