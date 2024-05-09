package frc.robot.commands.swervedrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveDistance extends Command
{
  private final SwerveSubsystem swerveSubsystem;
  private Pose2d startPose;
  private Pose2d currentPose;
  private Pose2d endPose;
  

  public DriveDistance(SwerveSubsystem swerveSubsystem)
    {  
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);  
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    startPose = swerveSubsystem.getPose();
    // Calculate the new pose
    double newX = startPose.getTranslation().getX() + 1 * Math.cos(startPose.getRotation().getRadians());
    double newY = startPose.getTranslation().getY() + 1 * Math.sin(startPose.getRotation().getRadians());
    endPose = new Pose2d(newX, newY, startPose.getRotation());
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    currentPose = swerveSubsystem.getPose();
    // Create a command to drive to the new pose
    swerveSubsystem.drive(new Translation2d(endPose.getX() - currentPose.getX(), endPose.getY() - currentPose.getY()),
                                          0,
                                          false);
  }

  @Override
  public boolean isFinished()
  {

    // Check if the robot has reached the end pose within a certain tolerance
    return Math.hypot(endPose.getTranslation().getX() - currentPose.getTranslation().getX(),
                      endPose.getTranslation().getY() - currentPose.getTranslation().getY()) < 0.1;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.lock();
  }

}