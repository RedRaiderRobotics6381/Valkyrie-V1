package frc.robot.commands.Vision;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonUtils;
// import org.photonvision.PhotonVersion;
// import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LEDsSubSystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PickUpNoteCmd extends Command
{
  private final SwerveSubsystem swerveSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LauncherRotateSubsystem m_launcherRotateSubsystem;
  private final PIDController   xController;
  //private final PIDController   yController;
  private final PIDController   zController;
  private boolean hasTargets;
  private boolean droveToNote;
  private double TX = 0;
  private double TZ = 0;
  
  int peakVelocity;
  int currentVelocity;

  boolean lowerIntakeHasNote;
  boolean hasNote;
  boolean outtakeHasNote;
  boolean intakeHasNote;

  public PickUpNoteCmd(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, LauncherRotateSubsystem launcherRotateSubsystem)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_launcherRotateSubsystem = launcherRotateSubsystem;
    
    xController = new PIDController(0.15, 0.00, 0.0);
    //yController = new PIDController(0.0625, 0.00375, 0.0001);
    zController = new PIDController(0.015,0.0, 0.000);
    xController.setTolerance(3);
    //yController.setTolerance(3);
    zController.setTolerance(.5);

    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(swerveSubsystem, intakeSubsystem, launcherRotateSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    droveToNote = false;
    hasNote = false;
    TX = 0;
  }

    /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    outtakeHasNote = Robot.sensorOuttake.get(); //Check if the note is in the outtake
    //intakeHasNote = m_intakeSubsystem.colorSensor.getProximity() > 1500;
    
    intakeHasNote = Robot.sensorIntake.get(); //Check if the note is in the intake
    //intakeHasNote = Robot.colorSensorIntake.getProximity() > 1500;  //Tune the proximity value to be where the note is found.
    var result = Robot.camObj.getLatestResult();  // Get the latest result from PhotonVision
    hasTargets = result.hasTargets(); // Check if the latest result has any targets.
    PhotonTrackedTarget target = result.getBestTarget();
    if (!outtakeHasNote){
      if (hasTargets == true) { // && RobotContainer.driverXbox.getRawButton(2) == true
        TZ = target.getYaw();
        
        //The code below is to track a single note instead of sometimes driving towards a note further ahead that it might see.
        if (TX == 0) { //If the target pitch is 0 (the first time the code runs), set the target pitch to the current pitch
          TX = target.getPitch();
        } else if (target.getPitch() < TX) {
          TX = target.getPitch(); //If the target pitch is less than the current pitch, set the target pitch to the current pitch
        }


        double translationValx = MathUtil.clamp(-xController.calculate(TX, -14), -4.0 , 4.0); //Tune the setpoint to be where the note is just barely found.
        double translationValz = MathUtil.clamp(zController.calculate(TZ, 0.0), -2.0 , 2.0); //* throttle, 2.5 * throttle);
        if (xController.atSetpoint() != true) {
            swerveSubsystem.drive(new Translation2d(translationValx, 0.0), translationValz, false);
          } else {
        

        if (!intakeHasNote && !droveToNote){ //If the note is not in the intake, run the intake command
                m_launcherRotateSubsystem.launcherRotatePIDController.setReference(LauncherConstants.posIntake,CANSparkMax.ControlType.kSmartMotion);
                m_intakeSubsystem.intakeMotor.set(IntakeConstants.intakeSpeed);

                m_intakeSubsystem.indexerMotor.set(IntakeConstants.indexerIntakeSpeed);
                m_intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.launcherIndexerIntakeSpeed);
                //swerveSubsystem.drive(new Translation2d(0.5, 0.0), 0.0, false);
                swerveSubsystem.driveToPose(
                              new Pose2d(new Translation2d(
                                (swerveSubsystem.getPose().getX() + 0.5) * Math.cos(swerveSubsystem.getPose().getRotation().getRadians()),
                                (swerveSubsystem.getPose().getY() + 0.5) * Math.sin(swerveSubsystem.getPose().getRotation().getRadians())),
                                Rotation2d.fromDegrees(swerveSubsystem.getPose().getRotation().getDegrees())));
              }             // swerveSubsystem.drive(new Translation2d(0.5, 0.0), 0.0, false);
              if (IntakeSubsystem.colorSensor.getProximity() > 50){
                swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, false);
                //swerveSubsystem.lock();
                droveToNote = true;                
              }
            }
          
        } 
      } else{
        hasNote = true;
      }
      // if (!Robot.camObj.isConnected()) {
      //           swerveSubsystem.lock();
      //         }
      // //System.out.println(lowerIntakeHasNote);
      
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
    return hasNote;
    //return droveToNote;
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
    LEDsSubSystem.setLED(.91);
    m_intakeSubsystem.indexerMotor.set(IntakeConstants.zeroSpeed);
    m_intakeSubsystem.intakeMotor.set(IntakeConstants.zeroSpeed);
    m_intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.zeroSpeed);
    swerveSubsystem.lock();
    TX = 0; TZ = 0;
    droveToNote = false;
  }
}


