package frc.robot.commands.Vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.Secondary.LauncherSubsystem;
import edu.wpi.first.math.MathUtil;

public class LauncherAimAutonCMD extends Command
{
  public static double Launcher_Pitch;
  boolean aimedToTarget;
  
  private PhotonTrackedTarget lastTarget;
  private final LauncherRotateSubsystem m_launcherRotateSubsystem;
  private final LauncherSubsystem m_launcherSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;

  public LauncherAimAutonCMD(LauncherRotateSubsystem launcherRotateSubsystem, LauncherSubsystem launcherSubsystem, IntakeSubsystem intakeSubsystem)
  {
    this.m_launcherRotateSubsystem = launcherRotateSubsystem;
    this.m_launcherSubsystem = launcherSubsystem;
    this.m_intakeSubsystem = intakeSubsystem;
    addRequirements(launcherRotateSubsystem, launcherSubsystem, intakeSubsystem);
  }
  {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    
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
    aimedToTarget = false;

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
            LauncherConstants.LauncherSpeedMult = MathUtil.clamp(LAUNCHER_TO_TOWER * 1750, 2750, 4000);
            Double ID_HEIGHT = 2.21;//Meters
            Launcher_Pitch = ((Math.toDegrees(Math.atan(ID_HEIGHT / LAUNCHER_TO_TOWER))) + 90);
             m_launcherSubsystem.launcherPIDControllerTop.setReference(LauncherConstants.LauncherSpeedMult, CANSparkFlex.ControlType.kVelocity);
            m_launcherRotateSubsystem.launcherRotatePIDController.setReference(Launcher_Pitch,CANSparkMax.ControlType.kSmartMotion);
            if ((Math.abs(m_launcherRotateSubsystem.launcherRotateEncoder.getPosition() -
                 Launcher_Pitch) <= LauncherConstants.LauncherAngleTol+.5) && ((Math.abs(m_launcherSubsystem.launcherMotorTop.getEncoder().getVelocity() -
                 LauncherConstants.LauncherSpeedMult)) <= LauncherConstants.LauncherSpeedTol+50)){
                m_intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.launcherIndexerOuttakeSpeed);
                m_intakeSubsystem.indexerMotor.set(IntakeConstants.indexerOuttakeSpeed);
              } 
            }
            
            // SmartDashboard.putNumber("Angle to Target", Launcher_Pitch);
            // SmartDashboard.putNumber("Dist to Target", LAUNCHER_TO_TOWER);

          
        } 
      } else {
        aimedToTarget = true;
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
    return aimedToTarget;
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
    m_intakeSubsystem.indexerMotor.set(0);
    m_intakeSubsystem.launcherIndexerMotor.set(0);
    m_launcherSubsystem.launcherPIDControllerTop.setReference(0, CANSparkFlex.ControlType.kVelocity);
    m_launcherRotateSubsystem.launcherRotateMotor.disable();
  }
}
