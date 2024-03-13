// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Secondary.ClimberSubsystem;

public class LowerCmd extends Command {

  private final ClimberSubsystem climberSubsystem;
  private boolean lowered = false;
  private double lowerDist = 13.0;
  private double lowerStopDist = 0.5;

  public LowerCmd(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;

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
if(Robot.rightLimitSwitch.get()){
    if (climberSubsystem.m_climberEncoderR.getPosition() >= lowerStopDist && climberSubsystem.m_climberEncoderR.getPosition() <= lowerDist){
      climberSubsystem.m_climberMotorR.set(-.75);
    }
  }
    if (climberSubsystem.m_climberEncoderR.getPosition() <= lowerStopDist) {
      climberSubsystem.m_climberMotorR.set(0);
    }
  
  if(Robot.leftLimitSwitch.get()){
    if (climberSubsystem.m_climberEncoderL.getPosition() >= lowerStopDist && climberSubsystem.m_climberEncoderL.getPosition() <= lowerDist){
      climberSubsystem.m_climberMotorL.set(-.75);
    } 
  }
    if (climberSubsystem.m_climberEncoderL.getPosition() <= lowerStopDist) {
      climberSubsystem.m_climberMotorL.set(0);
    }
  
    if (climberSubsystem.m_climberEncoderR.getPosition() <= lowerStopDist && climberSubsystem.m_climberEncoderL.getPosition() <= lowerStopDist){
    lowered = true;
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
    return lowered;
  }
}
