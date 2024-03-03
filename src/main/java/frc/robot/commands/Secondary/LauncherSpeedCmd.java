// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.LauncherSubsystem;

public class LauncherSpeedCmd extends Command {
  
  private final LauncherSubsystem launcherSubsystem;
  private final double launcherSpeedSetpoint;
  public static boolean speedComplete = false;
  
  /** Creates a new LauncherRotateCmd. */
  public LauncherSpeedCmd(double launcherSpeedSetpoint, LauncherSubsystem launcherSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcherSpeedSetpoint = launcherSpeedSetpoint;
    this.launcherSubsystem = launcherSubsystem;
    addRequirements(this.launcherSubsystem);
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    speedComplete = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcherSubsystem.launcherPIDControllerTop.setReference(launcherSpeedSetpoint, CANSparkMax.ControlType.kVelocity);
    if (launcherSubsystem.getLauncherSpeed() == launcherSpeedSetpoint) {
      speedComplete = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return speedComplete;
  }
}
