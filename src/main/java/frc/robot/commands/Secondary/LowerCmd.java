// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ClimberSubsystem;


public class LowerCmd extends Command {

  private final ClimberSubsystem climberSubsystem;
  private boolean lowered = false;
  private double lowerDist = 11.0;
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

    /*
    Here's a breakdown of the if-else logic:

    The first if statement checks if the position of the right lowering motor is between -0.01 and lowerDist (inclusive).
    If it is, the right lowering motor is set to run at 25% speed in the reverse direction (indicated by the negative sign).

    The second if statement checks if the position of the right lowering motor is less than or equal to lowerStopDist.
    If it is, the right lowering motor is stopped.

    The third if statement checks if the position of the left lowering motor is between -0.01 and lowerDist (inclusive).
    If it is, the left lowering motor is set to run at 25% speed in the reverse direction.

    The fourth if statement checks if the position of the left lowering motor is less than or equal to lowerStopDist.
    If it is, the left lowering motor is stopped.

    The final if statement checks if the positions of both the right and left lowering motors are less than or equal to lowerStopDist.
    If they are, the lowered boolean variable is set to true.
    */

    if (climberSubsystem.m_climberEncoderR.getPosition() >= -0.01 && climberSubsystem.m_climberEncoderR.getPosition() <= lowerDist){
      climberSubsystem.m_climberMotorR.set(-.25);
    }
    if (climberSubsystem.m_climberEncoderR.getPosition() <= lowerStopDist) {
      climberSubsystem.m_climberMotorR.set(0);
    }

    if (climberSubsystem.m_climberEncoderL.getPosition() >= -0.01 && climberSubsystem.m_climberEncoderL.getPosition() <= lowerDist){
      climberSubsystem.m_climberMotorL.set(-.25);
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

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lowered;
  }
}
