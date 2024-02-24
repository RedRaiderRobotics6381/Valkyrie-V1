// package frc.robot.commands.Vision;

// import org.photonvision.targeting.PhotonTrackedTarget;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.Constants.AprilTagConstants;
// import frc.robot.Constants.LauncherConstants;
// //import frc.robot.commands.Secondary.Speaker;
// import frc.robot.subsystems.Secondary.IntakeSubsystem;
// import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
// import frc.robot.subsystems.Secondary.LauncherSubsystem;
// import edu.wpi.first.math.util.Units;

// public class LauncherAimCMD extends Command
// {
//   public static double Launcher_Pitch;
//   private final LauncherRotateSubsystem launcherRotateSubsystem;
//   private final LauncherSubsystem launcherSubsystem;
//   private final IntakeSubsystem intakeSubsystem;
//   private PhotonTrackedTarget lastTarget;

//   public LauncherAimCMD(LauncherRotateSubsystem launcherRotateSubsystem, LauncherSubsystem launcherSubsystem , IntakeSubsystem intakeSubsystem)
//   {

//     // each subsystem used by the command must be passed into the
//     // addRequirements() method (which takes a vararg of Subsystem)
//     this.launcherRotateSubsystem = launcherRotateSubsystem;
//     this.launcherSubsystem = launcherSubsystem;
//     this.intakeSubsystem = intakeSubsystem;
//   }

//   /**
//    * The initial subroutine of a command.  Called once when the command is initially scheduled.
//    */
//   @Override
//   public void initialize()
//   {
//     lastTarget = null;
//   }

//   /**
//    * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
//    * until {@link #isFinished()}) returns true.)
//    */
//   @Override
//   public void execute()
//   {
//     if (Robot.sensorOuttake.get() == true){
//       var photonRes = Robot.camAprTgLow.getLatestResult();
//       //System.out.println(photonRes.hasTargets());
//       if (photonRes.hasTargets()) {
//           //Find the tag we want to chase
//           var targetOpt = photonRes.getTargets().stream()
//             .filter(t -> t.getFiducialId() == AprilTagConstants.speakerID) //4 Red & 7 Blue
//             .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() != -1)
//             .findFirst();
//           if (targetOpt.isPresent()) {
//             var target = targetOpt.get();
//             // This is new target data, so recalculate the goal
//             lastTarget = target;
//             Double LAUNCHER_TO_TOWER = target.getBestCameraToTarget().getZ();
//             Double ID_HEIGHT = Units.inchesToMeters(57.13) - LauncherConstants.HEIGHT_TO_ROTATE_MOTOR;
//             Launcher_Pitch = Math.asin(ID_HEIGHT / (Math.sqrt(Math.pow(ID_HEIGHT,2) + Math.pow(LAUNCHER_TO_TOWER,2))));
//             launcherRotateSubsystem.rotatePosCommand(Launcher_Pitch);
//             SmartDashboard.putNumber("Angle to Target", Launcher_Pitch);
//             // if (LAUNCHER_TO_TOWER <= 6){ 
//             //   if (target.getYaw() >= -2  || target.getYaw() <=2){
//             //     new Speaker(intakeSubsystem, launcherSubsystem);
//             //   }
//             // }
//           }
//         } 
//       } 
//     }
   

//   /**
//    * <p>
//    * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
//    * the scheduler will call its {@link #end(boolean)} method.
//    * </p><p>
//    * Returning false will result in the command never ending automatically. It may still be cancelled manually or
//    * interrupted by another command. Hard coding this command to always return true will result in the command executing
//    * once and finishing immediately. It is recommended to use *
//    * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
//    * </p>
//    *
//    * @return whether this command has finished.
//    */
//   @Override
//   public boolean isFinished()
//   {
//     return Robot.sensorOuttake.get() == false;
//   }

//   /**
//    * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
//    * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
//    * up loose ends, like shutting off a motor that was being used in the command.
//    *
//    * @param interrupted whether the command was interrupted/canceled
//    */
//   @Override
//   public void end(boolean interrupted)
//   {
//     //launcherSubsystem.LauncherCmd(0);
//   }
// }
