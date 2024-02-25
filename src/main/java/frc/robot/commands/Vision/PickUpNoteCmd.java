package frc.robot.commands.Vision;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Secondary.IntakeCmd;
//import frc.robot.commands.Secondary.IntakeCmd;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class PickUpNoteCmd extends Command
{
  private final SwerveSubsystem swerveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final LauncherRotateSubsystem launcherRotateSubsystem;
  private final PIDController   xController;
  private final PIDController   yController;
  private final PIDController   zController;
  private boolean hasTargets;
  private boolean hasNote;
  private boolean droveToNote;

  //public PickUpNoteCmd(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, LauncherRotateSubsystem launcherRotateSubsystem)
  public PickUpNoteCmd(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, LauncherRotateSubsystem launcherRotateSubsystem)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.launcherRotateSubsystem = launcherRotateSubsystem;
    xController = new PIDController(0.055, 0.00, 0.0);
    //xController = new PIDController(0.0625, 0.00375, 0.2);
    yController = new PIDController(0.0625, 0.00375, 0.0001);
    zController = new PIDController(0.025,0.0, 0.000);
    xController.setTolerance(1);
    yController.setTolerance(1);
    zController.setTolerance(.5);

    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    droveToNote = false;
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    var result = Robot.camObj.getLatestResult();  // Get the latest result from PhotonVision
    hasTargets = result.hasTargets(); // Check if the latest result has any targets.
    PhotonTrackedTarget target = result.getBestTarget();
    
    if (hasTargets == true && RobotContainer.driverXbox.getRawButton(2) == true) {
      double TZ = target.getYaw();
      double TX = target.getPitch();

      double translationValx = MathUtil.clamp(-xController.calculate(TX, -19), -1.0 , 1.0); //Tune the setpoint to be where the note is just barely found.
      double translationValz = MathUtil.clamp(zController.calculate(TZ, 0.0), -2.0 , 2.0); //* throttle, 2.5 * throttle);

      if (xController.atSetpoint() != true) {
          swerveSubsystem.drive(new Translation2d(translationValx, 0.0), translationValz, false);
          //new IntakeSubsystem().IntakeCmd();
        } else{
          //swerveSubsystem.getPose();
          if (droveToNote == false){
            new IntakeCmd(intakeSubsystem, launcherRotateSubsystem);
            //swerveSubsystem.drive(new Translation2d(0.5, 0.0), 0.0, false);
            swerveSubsystem.driveToPose(new Pose2d(new Translation2d(0.25, 0.0), Rotation2d.fromDegrees(0)));
            droveToNote = true;
          }
      }
      hasNote = Robot.sensorIntake.get();
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
    //return hasNote;
      //Try this to see if we can use it as an end.  If the note is not intaked first we will probably need to add a drive to pose.
    //return xController.atSetpoint();// && yController.atSetpoint();
    return droveToNote;
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
    swerveSubsystem.lock();
    
    
    //RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kBothRumble, 0);
    //swerveSubsystem.lock();
  }
}