package frc.robot.commands.Vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.subsystems.Secondary.LEDsSubSystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import edu.wpi.first.math.MathUtil;

public class LauncherAimCMD extends Command
{
  public static double Launcher_Pitch;
  public static double LauncherSpeedMult;
  
  private PhotonTrackedTarget lastTarget;

  public LauncherAimCMD()
  {

    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    
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
    if (Robot.sensorOuttake.get() == true){
      var photonRes = Robot.camAprTgLow.getLatestResult();
      //System.out.println(photonRes.hasTargets());
      if (photonRes.hasTargets()) {
          //Find the tag we want to chase
          var targetOpt = photonRes.getTargets().stream()
            .filter(t -> t.getFiducialId() == AprilTagConstants.speakerID) //4 Red & 7 Blue
            .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() != -1)
            .findFirst();
          if (targetOpt.isPresent()) {
            var target = targetOpt.get();
            // This is new target data, so recalculate the goal
            lastTarget = target;
            double targetX = target.getBestCameraToTarget().getX();
            Double targetY = target.getBestCameraToTarget().getY();
            Double LAUNCHER_TO_TOWER = Math.sqrt(Math.pow(targetX, 2) + Math.pow(targetY, 2));
            //Double LAUNCHER_TO_TOWER = target.getBestCameraToTarget().getX();
            LauncherSpeedMult = MathUtil.clamp(LAUNCHER_TO_TOWER * 600, 1800, 4000);
            Double ID_HEIGHT = 2.775;//Meters
            Launcher_Pitch = ((Math.toDegrees(Math.atan(ID_HEIGHT / LAUNCHER_TO_TOWER))) + 90);
            LauncherRotateSubsystem.m_LauncherRotatePIDController.setReference(Launcher_Pitch,CANSparkMax.ControlType.kSmartMotion);
            SmartDashboard.putNumber("Angle to Target", Launcher_Pitch);
            SmartDashboard.putNumber("Dist to Target", LAUNCHER_TO_TOWER);

            if (LAUNCHER_TO_TOWER <= 5){ 
              if (target.getYaw() >= -2  || target.getYaw() <=2){
                LEDsSubSystem.setLEDwBlink(.73,.125);
              }
            }
          }
        } 
      } 
    }
   

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return Robot.sensorOuttake.get() == false;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    //launcherSubsystem.LauncherCmd(0);
  }
}
