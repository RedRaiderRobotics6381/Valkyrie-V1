// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LEDsSubSystem;

public class OuttakeCmd extends Command {

  private boolean hasNote = true;
  private final IntakeSubsystem intakeSubsystem;
  
  public OuttakeCmd(IntakeSubsystem intakeSubsystem){
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasNote = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.sensorOuttake.get() == true){
      intakeSubsystem.indexerMotor.set(IntakeConstants.indexerOuttakeSpeed);
      intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.indexerOuttakeSpeed);
    } else{
      hasNote = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LEDsSubSystem.setLED(.99);
    intakeSubsystem.indexerMotor.set(IntakeConstants.zeroSpeed);
    intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.zeroSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasNote;
  }
}