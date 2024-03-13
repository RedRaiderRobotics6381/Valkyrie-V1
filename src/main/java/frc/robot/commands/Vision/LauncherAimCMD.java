package frc.robot.commands.Vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import edu.wpi.first.math.MathUtil;

public class LauncherAimCMD extends Command
{
  public static double Launcher_Pitch;
  public static double ID_HEIGHT = 2.19;//2.1936
  public boolean finishedAiming;
  
  
  private PhotonTrackedTarget lastTarget;
  private final LauncherRotateSubsystem m_LauncherRotateSubsystem;

  public LauncherAimCMD(LauncherRotateSubsystem launcherRotateSubsystem)
  {
    this.m_LauncherRotateSubsystem = launcherRotateSubsystem;
    addRequirements(launcherRotateSubsystem);
  }
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
    finishedAiming = false;
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
            double targetX = target.getBestCameraToTarget().getX() - 0.233; //0.233
            double targetY = target.getBestCameraToTarget().getY() + 0.306; //0.306
            double LAUNCHER_TO_TOWER = Math.sqrt(Math.pow(targetX, 2) + Math.pow(targetY, 2));
            
            //Double LAUNCHER_TO_TOWER = target.getBestCameraToTarget().getX();
            LauncherConstants.LauncherSpeedMult = MathUtil.clamp(LAUNCHER_TO_TOWER * 1500, 2750, 4000);
            //Meters from 1.808 (2.1436) (2.2936(3/7/24))
            Launcher_Pitch = ((Math.toDegrees(Math.atan(ID_HEIGHT / LAUNCHER_TO_TOWER))) + 90);
            m_LauncherRotateSubsystem.launcherRotatePIDController.setReference(Launcher_Pitch,CANSparkMax.ControlType.kSmartMotion);
            if ((Math.abs(m_LauncherRotateSubsystem.launcherRotateEncoder.getPosition() -
                 Launcher_Pitch) <= LauncherConstants.LauncherAngleTol)){
              finishedAiming = true;
            }
            SmartDashboard.putNumber("Angle to Target", Launcher_Pitch);
            SmartDashboard.putNumber("Dist to Target", LAUNCHER_TO_TOWER);

            if (LAUNCHER_TO_TOWER <= 5){ 
              if (target.getYaw() >= -2  || target.getYaw() <=2){
                //LEDsSubSystem.setLEDwBlink(.73,.125); //Removed lights to stop alerting other teams we are ready to shoot.
              //  RobotContainer.driverXbox.setRumble(RumbleType.kBothRumble, 0.25);
              //  RobotContainer.driverXbox.setRumble(RumbleType.kBothRumble, 0.25);
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
    //return Robot.sensorOuttake.get() == false;
    return finishedAiming;
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
