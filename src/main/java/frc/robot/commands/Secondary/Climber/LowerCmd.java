// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ClimberSubsystem;

public class LowerCmd extends Command {

  private final ClimberSubsystem m_climberSubsystem;
  private boolean lowered = false;
  private boolean loweredL = false;
  private boolean loweredR = false;
  //private double lowerDist = 13.0;

  public LowerCmd(ClimberSubsystem climberSubsystem) {
    this.m_climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    lowered = false;
    loweredR = false;
    loweredL = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){

  // if climber position is LESS THAN 12        
  // then set climber speed to -.75

      //else if climber position is LESS THAN 13
      //then set climber speed to -.25
          
         // else set climber speed to 0
      
      if (Math.abs(m_climberSubsystem.climberEncoderR.getPosition()) < 10) {
        m_climberSubsystem.climberMotorR.set(-.75);
      } else if (Math.abs(m_climberSubsystem.climberEncoderR.getPosition()) < 13){
        m_climberSubsystem.climberMotorR.set(-.25);
      } else {
        m_climberSubsystem.climberMotorR.set(0);
        loweredR = true;
      }
    if (Math.abs(m_climberSubsystem.climberEncoderL.getPosition()) < 10) {
      m_climberSubsystem.climberMotorL.set(-.75);
    } else if (Math.abs(m_climberSubsystem.climberEncoderL.getPosition()) < 13) {
      m_climberSubsystem.climberMotorL.set(-.25);
    } else {
      m_climberSubsystem.climberMotorL.set(0);
      loweredL = true;
    }
    
        // if both climber arms are lowered
        // then set lowered to true

    if (loweredR == true && loweredL == true) {
      lowered = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.climberMotorL.set(0);
    m_climberSubsystem.climberMotorR.set(0);
    if(interrupted){
      m_climberSubsystem.climberMotorL.set(0);
      m_climberSubsystem.climberMotorR.set(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lowered;    
  }
}
