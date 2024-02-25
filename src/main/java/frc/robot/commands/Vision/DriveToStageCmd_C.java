package frc.robot.commands.Vision;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToStageCmd_C extends Command
{
  private final SwerveSubsystem swerveSubsystem;
  private static final Transform3d TAG_TO_GOAL = new Transform3d(
                                                                 new Translation3d(1.15, 0, 0),
                                                                 new Rotation3d(0.0,0.0,Math.PI));
  
  private final Supplier<Pose2d> poseProvider;

  private PhotonTrackedTarget lastTarget;
  boolean atX;
  boolean atY;
  boolean atZ;
  boolean atGoal;
  public DriveToStageCmd_C(SwerveSubsystem swerveSubsystem)
    {  
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    this.swerveSubsystem = swerveSubsystem;
    this.poseProvider = swerveSubsystem::getPose;
    
    addRequirements(swerveSubsystem); 
        
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    lastTarget = null;
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {


    var robotPose2d = poseProvider.get();
    var robotPose = new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

    var photonResLow = Robot.camAprTgLow.getLatestResult();
    var photonResHigh = Robot.camAprTgHigh.getLatestResult();
    var photonRes = photonResLow; // Default to low resolution result
    if (photonResLow.hasTargets()) {
      photonRes = Robot.camAprTgLow.getLatestResult();
    }
    if (photonResHigh.hasTargets()) {
      photonRes = Robot.camAprTgHigh.getLatestResult();
    }
  
    //System.out.println(photonRes.hasTargets());
    if (photonRes.hasTargets()) {
      //Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
      .filter(t -> t.getFiducialId() == AprilTagConstants.stageIDA ||
                   t.getFiducialId() == AprilTagConstants.stageIDB ||
                   t.getFiducialId() == AprilTagConstants.stageIDC) //Red 11,12,13 & Blue 14,15,16 - 12 & 12 face the amps
      .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() != -1)
      .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;

        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose
            .transformBy(new Transform3d(new Translation3d(0.0, 0.0, -0.558), new Rotation3d())); //-.475

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        

        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
        
        if (Math.abs(goalPose.getX()) <= .1){
          atX = true;
        }
        if (Math.abs(goalPose.getY()) <= .1){
          atY = true;
        }
        if (Math.abs(goalPose.getRotation().getDegrees()) <= 3){
          atZ = true;
        }

        if (!atX || !atY){
          swerveSubsystem.driveToPose(new Pose2d(new Translation2d(goalPose.getX(), goalPose.getY()), goalPose.getRotation()));
        } else {
          atGoal = true;
        }

      }
    }

    if (lastTarget == null) {
      // No target has been visible
      // swerveSubsystem.lock();
    } else {
      
    }
  }

  @Override
  public boolean isFinished(){
    return atGoal;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.lock();
  }

}