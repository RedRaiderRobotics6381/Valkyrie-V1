package frc.robot.commands.Vision;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToStageCmd extends Command
{
  private final SwerveSubsystem swerveSubsystem;
  
  private final PIDController   xController;
  private final PIDController   yController;
  private final PIDController   omegaController;
  
  private double xTol = 0.1; //meters
  private double yTol = 0.05; //meters
  private double omegaTol = 0.05;//Math.toRadians(1.0);
  
  private static double xOffset = 1.3; //meters
  private static double yOffset = 0.0; //meters
  private static double omegaOffset = 0.00; //degrees
  
  private double TX;
  private double TY;
  private double TZ;
  
  private static boolean atSetpoint;
  
  private PhotonTrackedTarget lastTarget;


  public DriveToStageCmd(SwerveSubsystem swerveSubsystem)
    {  
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    this.swerveSubsystem = swerveSubsystem;
    
    //xController = new PIDController(0.055, 0.00, 0.0);
    xController = new PIDController(2.0, 0.0, 0.0);
    yController = new PIDController(1.0, 0.0, 0);
    // xController = new PIDController(0.0625, 0.00375, 0.2);
    // yController = new PIDController(0.0625, 0.00375, 0.0001);
    //omegaController = new PIDController(0.125,0.000, 0.0); //p from 0.0625
    omegaController = new PIDController(1.2, 3, 0.0675); //.7 is taken from default pathplanner auton constants which also works in radians.
    xController.setTolerance(xTol); //meters
    yController.setTolerance(yTol); //meters
    omegaController.setTolerance(omegaTol);
    xController.setSetpoint(xOffset); //meters
    yController.setSetpoint(yOffset); //meters
    omegaController.setSetpoint(omegaOffset); //degrees
   // omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
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
    // Put the initial PID values on the SmartDashboard
    SmartDashboard.putNumber("OP", omegaController.getP());
    SmartDashboard.putNumber("OI", omegaController.getI());
    SmartDashboard.putNumber("OD", omegaController.getD());

    SmartDashboard.putNumber("OP", omegaController.getP());
    SmartDashboard.putNumber("OI", omegaController.getI());
    SmartDashboard.putNumber("OD", omegaController.getD());



  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {

    var photonResLow = Robot.camAprTgLow.getLatestResult();
    var photonResHigh = Robot.camAprTgHigh.getLatestResult();
    var photonRes = photonResLow; // Default to low resolution result
    // In your periodic method (like robotPeriodic or execute in a command), update the PID values from the SmartDashboard
    double p = SmartDashboard.getNumber("DtS Omega P", 0.0);
    double i = SmartDashboard.getNumber("DtS Omega I", 0.0);
    double d = SmartDashboard.getNumber("DtS Omega D", 0.0);
    omegaController.setPID(p, i, d);
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

        TX = target.getBestCameraToTarget().getX();
        TY = target.getBestCameraToTarget().getY();
        TZ = target.getYaw();//Math.toRadians(target.getYaw());
      }
    }

    if (lastTarget == null) {
      // No target has been visible
      swerveSubsystem.lock();
    } else {
      // Drive to the target
      double translationValx = MathUtil.clamp(xController.calculate(TX, xOffset), -1 , 1);
      double translationValy = MathUtil.clamp(yController.calculate(TY, yOffset), -1 , 1);
      double translationValz = MathUtil.clamp(omegaController.calculate(TZ, omegaOffset), -1 , 1);
      //|| omegaController.atSetpoint() != true
      if (xController.atSetpoint() != true || yController.atSetpoint() != true || omegaController.atSetpoint() != true){
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