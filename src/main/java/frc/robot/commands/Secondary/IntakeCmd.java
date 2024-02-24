// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Secondary;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.LauncherConstants;
// import frc.robot.subsystems.Secondary.IntakeSubsystem;
// import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;

// public class IntakeCmd extends Command {

//   private final IntakeSubsystem intakeSubsystem;
//   private final LauncherRotateSubsystem launcherRotateSubsystem;
//   private boolean hasNote = false;

//   public IntakeCmd(IntakeSubsystem intakeSubsystem , LauncherRotateSubsystem launcherRotateSubsystem) {
//     this.intakeSubsystem = intakeSubsystem;
//     this.launcherRotateSubsystem = launcherRotateSubsystem;
    
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     hasNote = false;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(Robot.sensorOuttake.get() == true){
//       hasNote = true;
//     } else {
//       launcherRotateSubsystem.rotatePosCommand(LauncherConstants.posOuttake);
//       intakeSubsystem.indexerMotor.set(IntakeConstants.indexerSpeed);
//       intakeSubsystem.intakeMotor.set(IntakeConstants.intakeSpeed);
//       intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.launcherIndexerSpeed);
//       System.out.println(Robot.sensorIntake.get());
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     intakeSubsystem.indexerMotor.set(IntakeConstants.zeroSpeed);
//     intakeSubsystem.intakeMotor.set(IntakeConstants.zeroSpeed);
//     intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.zeroSpeed);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return hasNote;
//   }
// }
