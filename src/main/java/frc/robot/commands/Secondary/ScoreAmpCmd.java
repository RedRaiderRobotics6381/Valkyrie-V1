// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Secondary;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.LauncherConstants;
// import frc.robot.subsystems.Secondary.IntakeSubsystem;
// import frc.robot.subsystems.Secondary.LauncherSubsystem;

// public class ScoreAmpCmd extends Command {
//   /** Creates a new Outtake. */
  
//     private final IntakeSubsystem intakeSubsystem;
//     private final LauncherSubsystem launcherSubsystem;
//     private boolean hasNote = true;
  
  
//     public ScoreAmpCmd(IntakeSubsystem intakeSubsystem , LauncherSubsystem launcherSubsystem) {
//       this.intakeSubsystem = intakeSubsystem;
//       this.launcherSubsystem = launcherSubsystem;
      
//       // Use addRequirements() here to declare subsystem dependencies.
//     }
  
//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//       hasNote = true;
//     }
  
//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//       if(Robot.sensorOuttake.get() == true || Robot.sensorIntake.get() == true){
//         launcherSubsystem.m_launcherMotorTop.set(LauncherConstants.launcherMotorTopSpeed);
//         if(launcherSubsystem.m_launcherMotorTop.getEncoder().getVelocity() >= 900) {
//           intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.launcherIndexerSpeed);
//           intakeSubsystem.indexerMotor.set(IntakeConstants.indexerSpeed);
//         }
//       } else {
//         hasNote = false;
//       }
//     }
  
//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//       intakeSubsystem.indexerMotor.set(IntakeConstants.zeroSpeed);
//       intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.zeroSpeed);
//       launcherSubsystem.m_launcherMotorTop.set(IntakeConstants.zeroSpeed);
//     }
  
//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//       return !hasNote;
//     }
//   }

