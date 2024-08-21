// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LEDsSubSystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.Secondary.LauncherSubsystem;

public class RevIntakeCmd extends Command {

  
  private boolean hasNote = false;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LauncherRotateSubsystem m_launcherRotateSubsystem;
  private final LauncherSubsystem m_launcherSubsystem;

  public RevIntakeCmd(IntakeSubsystem intakeSubsystem, LauncherRotateSubsystem launcherRotateSubsystem, LauncherSubsystem launcherSubsystem) {
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_launcherRotateSubsystem = launcherRotateSubsystem;
    this.m_launcherSubsystem = launcherSubsystem;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, launcherRotateSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasNote = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(Robot.sensorOuttake.get() == true || Robot.sensorIntake.get() == true){
    //   hasNote = true;
    // } else {
      m_launcherRotateSubsystem.launcherRotatePIDController.setReference(LauncherConstants.posIntake,CANSparkMax.ControlType.kSmartMotion);
      m_intakeSubsystem.indexerMotor.set(-IntakeConstants.indexerIntakeSpeed);
      m_intakeSubsystem.intakeMotor.set(-IntakeConstants.intakeSpeed);
      m_launcherSubsystem.launcherPIDControllerTop.setReference(1000, CANSparkFlex.ControlType.kVelocity);
      //m_intakeSubsystem.launcherIndexerMotor.set(-IntakeConstants.launcherIndexerIntakeSpeed);
      //System.out.println(Robot.sensorIntake.get());

    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LEDsSubSystem.setLED(.91);
    m_intakeSubsystem.indexerMotor.set(IntakeConstants.zeroSpeed);
    m_intakeSubsystem.intakeMotor.set(IntakeConstants.zeroSpeed);
    m_intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.zeroSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasNote;
  }
}
