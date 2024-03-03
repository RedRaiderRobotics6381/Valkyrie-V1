// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.Secondary.LauncherSubsystem;

public class ScoreCmd extends Command {
  /** Creates a new Outtake. */
  
    
    private final LauncherSubsystem launcherSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LauncherRotateSubsystem launcherRotateSubsystem;
    private final double LauncherAngle;
    private final double LauncherSpeed;

    private boolean hasNote = true;
  
  
    public ScoreCmd(double LauncherAngle, double LauncherSpeed, LauncherSubsystem launcherSubsystem, IntakeSubsystem intakeSubsystem, LauncherRotateSubsystem launcherRotateSubsystem) {
      this.launcherSubsystem = launcherSubsystem;
      this.intakeSubsystem = intakeSubsystem;
      this.launcherRotateSubsystem = launcherRotateSubsystem;
      this.LauncherAngle = LauncherAngle;
      this.LauncherSpeed = LauncherSpeed;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(this.launcherSubsystem, this.intakeSubsystem, this.launcherRotateSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      hasNote = true;
    }


    // // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if(Robot.sensorOuttake.get() == true || Robot.sensorIntake.get() == true){
        new LauncherRotateCmd(LauncherAngle, launcherRotateSubsystem);
        new LauncherSpeedCmd(LauncherSpeed, launcherSubsystem);
        new WaitUntilCommand(() -> LauncherRotateCmd.rotateComplete).withTimeout(2);
        new WaitUntilCommand(() -> LauncherSpeedCmd.speedComplete).withTimeout(1);
        new OuttakeCmd(intakeSubsystem);
      } else {
        new WaitCommand(1.0);
        hasNote = false;
      }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      new LauncherSpeedCmd(0, launcherSubsystem);
      new LauncherRotateCmd(LauncherConstants.posIntake, launcherRotateSubsystem);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return !hasNote;
    }
  }

