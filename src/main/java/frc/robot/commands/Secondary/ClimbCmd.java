// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ClimberSubsystem;


public class ClimbCmd extends Command {

  private final ClimberSubsystem climberSubsystem;
  private boolean climbed = false;
  private double climbDist = 12.0;

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
    /*
    Here's a breakdown of the if-else logic:
      The first if statement checks if the position of the right climber motor is between -0.1 and climbDist (inclusive).
      If it is, the right climber motor is set to run at 25% speed.

      The second if statement checks if the position of the right climber motor is greater than or equal to climbDist.
      If it is, the right climber motor is stopped.

      The third if statement checks if the position of the left climber motor is between -0.1 and climbDist (inclusive).
      If it is, the left climber motor is set to run at 25% speed.

      The fourth if statement checks if the position of the left climber motor is greater than or equal to climbDist.
      If it is, the left climber motor is stopped.

      The final if statement checks if the positions of both the right and left climber motors are greater than or equal to climbDist.
      If they are, the climbed boolean variable is set to true.
    */

    if (climberSubsystem.m_climberEncoderR.getPosition() >= -0.1 && climberSubsystem.m_climberEncoderR.getPosition() <= climbDist){
        climberSubsystem.m_climberMotorR.set(.25);
    }
    if (climberSubsystem.m_climberEncoderR.getPosition() >= climbDist) {
        climberSubsystem.m_climberMotorR.set(0);
    }

    if (climberSubsystem.m_climberEncoderL.getPosition() >= -0.1 && climberSubsystem.m_climberEncoderL.getPosition() <= climbDist - 1.0){
        climberSubsystem.m_climberMotorL.set(.25);
      } 
    if (climberSubsystem.m_climberEncoderL.getPosition() >= climbDist - 1.0) {
        climberSubsystem.m_climberMotorL.set(0);
    }
    if (climberSubsystem.m_climberEncoderR.getPosition() >= climbDist && climberSubsystem.m_climberEncoderL.getPosition() >= climbDist - 1.0){
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
