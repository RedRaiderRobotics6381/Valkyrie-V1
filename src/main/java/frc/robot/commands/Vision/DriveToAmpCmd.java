// package frc.robot.commands.Vision;
// import java.util.function.Supplier;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.AprilTagConstants;
// import frc.robot.Robot;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// public class DriveToAmpCmd extends Command
// {
//   private final SwerveSubsystem swerveSubsystem;
//   private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(.75, 1.0);
//   private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(.75, 1.0);
//   private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
//   private static final Transform3d TAG_TO_GOAL = new Transform3d(
//                                                                  new Translation3d(Units.inchesToMeters(36), 0, 0),
//                                                                  new Rotation3d(0.0,0.0,Math.PI));
  
//   private final Supplier<Pose2d> poseProvider;
//   private final ProfiledPIDController xController = new ProfiledPIDController(0.25, 0.01, 0, X_CONSTRAINTS);
//   private final ProfiledPIDController yController = new ProfiledPIDController(0.25, 0.01, 0, Y_CONSTRAINTS);
//   private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

//   private PhotonTrackedTarget lastTarget;
//     public DriveToAmpCmd(SwerveSubsystem swerveSubsystem)
//   {  
//     // each subsystem used by the command must be passed into the
//     // addRequirements() method (which takes a vararg of Subsystem)
//     this.swerveSubsystem = swerveSubsystem;
//     this.poseProvider = swerveSubsystem::getPose;
//     xController.setTolerance(0.2); //0.2 meters
//     yController.setTolerance(0.2); //0.2 meters
//     omegaController.setTolerance(3.0); //3 degrees
//     omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
//     addRequirements(swerveSubsystem); 
//   }

//   /**
//    * The initial subroutine of a command.  Called once when the command is initially scheduled.
//    */
//   @Override
//   public void initialize()
//   {
//     lastTarget = null;
//     var robotPose = poseProvider.get();
//     omegaController.reset(robotPose.getRotation().getRadians());
//     xController.reset(robotPose.getX());
//     yController.reset(robotPose.getY());
//   }

//   /**
//    * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
//    * until {@link #isFinished()}) returns true.)
//    */
//   @Override
//   public void execute()
//   {
//     var robotPose2d = poseProvider.get();
//     var robotPose = new Pose3d(
//         robotPose2d.getX(),
//         robotPose2d.getY(),
//         0.0,
//         new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

//     var photonResLow = Robot.camAprTgLow.getLatestResult();
//     var photonResHigh = Robot.camAprTgHigh.getLatestResult();
//     var photonRes = photonResLow; // Default to low resolution result

//     if (photonResLow.hasTargets()) {
//       photonRes = Robot.camAprTgLow.getLatestResult();
//     }

//     if (photonResHigh.hasTargets()) {
//       photonRes = Robot.camAprTgHigh.getLatestResult();
//     }

//     //System.out.println(photonRes.hasTargets());
//     if (photonRes.hasTargets()) {
//       //Find the tag we want to chase
//       var targetOpt = photonRes.getTargets().stream()
//       .filter(t -> t.getFiducialId() == AprilTagConstants.ampID) //5 Red & 6 Blue
//       .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() != -1)
//       .findFirst();
//       if (targetOpt.isPresent()) {
//         var target = targetOpt.get();
//         // This is new target data, so recalculate the goal
//         lastTarget = target;
//         // Transform the robot's pose to find the camera's pose
//         var cameraPose = robotPose
//             .transformBy(new Transform3d(new Translation3d(-.475, 0.0, -0.558), new Rotation3d()));

//         // Trasnform the camera's pose to the target's pose
//         var camToTarget = target.getBestCameraToTarget();
//         var targetPose = cameraPose.transformBy(camToTarget);

//         // Transform the tag's pose to set our goal
//         var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

//         // Drive
//         xController.setGoal(goalPose.getX());
//         yController.setGoal(goalPose.getY());
//         omegaController.setGoal(goalPose.getRotation().getRadians());
//       }
//     }

//     if (lastTarget == null) {
//       // No target has been visible
//       swerveSubsystem.lock();
//     } else {
//       // Drive to the target
    
//       double xSpeed = MathUtil.clamp(xController.calculate(robotPose.getX()), -1.25, 1.25); 
//       if (xController.atGoal()) {
//         xSpeed = 0;
//       }

//       double ySpeed = MathUtil.clamp(-yController.calculate(robotPose.getY()), -0.75, 0.75);
//       if (yController.atGoal()) {
//         ySpeed = 0;
//       }

//       double omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
//       if (omegaController.atGoal()) {

//         omegaSpeed = 0;
//       }
//       swerveSubsystem.drive(new Translation2d(-xSpeed, ySpeed),
//       omegaSpeed,
//       false);
//       //swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
//     }
//   }

//   @Override
//   public void end(boolean interrupted) {
//     swerveSubsystem.lock();
//   }
// }
