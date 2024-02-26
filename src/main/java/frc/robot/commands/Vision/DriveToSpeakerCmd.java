package frc.robot.commands.Vision;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToSpeakerCmd extends Command
{
  private final SwerveSubsystem swerveSubsystem;
//   private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3.0, 1.5);
//   private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3.0, 1.5);
//   private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
//   private static final Transform3d TAG_TO_GOAL = new Transform3d(
//                                                                  new Translation3d(1.35, 0, 0),
//                                                                  new Rotation3d(0.0,0.0,Math.PI));
  
//   private final Supplier<Pose2d> poseProvider;
//   private final ProfiledPIDController xController = new ProfiledPIDController(6.5, 1.5, 0, X_CONSTRAINTS);
//   private final ProfiledPIDController yController = new ProfiledPIDController(2.255, 0, 0, Y_CONSTRAINTS);
//   private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

//   private PhotonTrackedTarget lastTarget;

  private final PIDController   xController;
  private final PIDController   yController;
  private final PIDController   omegaController;
  
  private double xTol = 0.1; //meters
  private double yTol = 0.05; //meters
  private double omegaTol = 1.5; //degrees
  
  private static double xOffset = 1.70; //meters
  private static double yOffset = 0.0; //meters
  private static double omegaOffset = 0.0; //degrees
  
  private double TX;
  private double TY;
  private double TZ;
  
  private static boolean atSetpoint;
  
  private PhotonTrackedTarget lastTarget;

  public DriveToSpeakerCmd(SwerveSubsystem swerveSubsystem)
  {  
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    this.swerveSubsystem = swerveSubsystem;
    // this.poseProvider = swerveSubsystem::getPose;
    // xController.setTolerance(0.1); //0.2 meters
    // yController.setTolerance(0.1); //0.2 meters
    // omegaController.setTolerance(3.0); //3 degrees
    // omegaController.enableContinuousInput(-Math.PI, Math.PI);
    xController = new PIDController(2.25, 0.0, 0.0);
    yController = new PIDController(1.0, 0.0, 0); //1.575
    // xController = new PIDController(0.0625, 0.00375, 0.2);
    // yController = new PIDController(0.0625, 0.00375, 0.0001);
    omegaController = new PIDController(0.0625,0.00025, 0.01);
    xController.setTolerance(xTol); //meters
    yController.setTolerance(yTol); //meters
    omegaController.setTolerance(omegaTol); //degrees
    xController.setSetpoint(xOffset); //meters
    yController.setSetpoint(yOffset); //meters
    omegaController.setSetpoint(omegaOffset); //degrees
    //omegaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerveSubsystem); 
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    lastTarget = null;
    atSetpoint = false;
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    var photonResLow = Robot.camAprTgLow.getLatestResult();
   // var photonResHigh = Robot.camAprTgHigh.getLatestResult();
    var photonRes = photonResLow; // Default to low resolution result
    if (photonResLow.hasTargets()) {
      photonRes = Robot.camAprTgLow.getLatestResult();
    }
    // if (photonResHigh.hasTargets()) {
    //   photonRes = Robot.camAprTgHigh.getLatestResult();
    // }
  
    //System.out.println(photonRes.hasTargets());
    if (photonRes.hasTargets()) {
      //Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
      .filter(t -> t.getFiducialId() == AprilTagConstants.speakerID) //Red 11,12,13 & Blue 14,15,16 - 12 & 12 face the amps
      .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() != -1)
      .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        
        // This is new target data, so recalculate the goal
        lastTarget = target;

        TX = target.getBestCameraToTarget().getX();
        TY = target.getBestCameraToTarget().getY();
        TZ = target.getYaw();
      }
    }

    if (lastTarget == null) {
      // No target has been visible
      // swerveSubsystem.lock();
    } else {
      // Drive to the target
      double translationValx = MathUtil.clamp(xController.calculate(TX, xOffset), -1 , 1);
      double translationValy = MathUtil.clamp(yController.calculate(TY, yOffset), -1 , 1);
      double translationValz = MathUtil.clamp(omegaController.calculate(TZ, omegaOffset), -1 , 1);
      //|| omegaController.atSetpoint() != true
      if ( xController.atSetpoint() != true || yController.atSetpoint() != true){
        swerveSubsystem.drive(new Translation2d(translationValx, translationValy),
        translationValz,
        false);
      } else{
          atSetpoint = true;
        }
    }
  }
  @Override
  public boolean isFinished()
  {
    return atSetpoint;  //Try this to see if we can use it as an end.  If the note is not intaked first we will probably need to add a drive to pose.
    //return xController.atSetpoint();// && yController.atSetpoint();
  }
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.lock();
  }
}