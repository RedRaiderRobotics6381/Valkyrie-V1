// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;

// import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.commands.Secondary.ClimberInitCmd;
import frc.robot.subsystems.Secondary.ClimberSubsystem;
// import frc.robot.commands.Secondary.ClimberInitCmd;
// import frc.robot.subsystems.Secondary.ClimberSubsystem;
import frc.robot.subsystems.Secondary.LEDsSubSystem;
import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private ClimberSubsystem climberSubsystem;
  
  private Timer disabledTimer;
  
  

  public static PhotonCamera camObj = new PhotonCamera("camObj");
  public static PhotonCamera camAprTgLow = new PhotonCamera("camAprTgLow");
  public static PhotonCamera camAprTgHigh = new PhotonCamera("camAprTgHigh");
   

  public static DigitalInput sensorIntake = new DigitalInput(1); //This is the lower sensor, it will be true when a note is first intaked
  public static DigitalInput sensorOuttake = new DigitalInput(0); //This is the upper sensor, it will be true when a note is ready for outtake
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    camObj.setDriverMode(false);
    camAprTgHigh.setDriverMode(false);
    camAprTgLow.setDriverMode(false);
    DriverStation.silenceJoystickConnectionWarning(true); // Disable joystick connection warning
    
    // SmartDashboard.putNumber("Amp Angle", LauncherConstants.AmpScoreAngle);
    // SmartDashboard.putNumber("Amp Speed", LauncherConstants.AmpScoreSpeed);
    // SmartDashboard.putNumber("Speaker Angle", LauncherConstants.SpeakerScoreAngle);
    // SmartDashboard.putNumber("Speaker Speed", LauncherConstants.SpeakerScoreSpeed);
    // SmartDashboard.putNumber("Trap Angle", LauncherConstants.TrapScoreAngle);
    // SmartDashboard.putNumber("Trap Speed", LauncherConstants.TrapScoreSpeed);
    // SmartDashboard.putNumber("Auto Score Aim Height", LauncherConstants.kAutoScoreAimHeight);
    // SmartDashboard.putNumber("Auto Score Speed", LauncherConstants.kAutoScoreSpeed);
    // SmartDashboard.putNumber("Auto Score Speed Min", LauncherConstants.kAutoScoreSpeedMin);
    // SmartDashboard.putNumber("Auto Score Speed Max", LauncherConstants.kAutoScoreSpeedMax);


  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // double nAmpScoreAngle = SmartDashboard.getNumber("Amp Angle", 0);
    // double nAmpScoreSpeed = SmartDashboard.getNumber("Amp Speed", 0);
    // double nSpeakerScoreAngle = SmartDashboard.getNumber("Speaker Angle", 0);
    // double nSpeakerScoreSpeed = SmartDashboard.getNumber("Speaker Speed", 0);
    // double nTrapScoreAngle = SmartDashboard.getNumber("Trap Angle", 0);
    // double nTrapScoreSpeed = SmartDashboard.getNumber("Trap Speed", 0);
    // double nAutoScoreAimHeight = SmartDashboard.getNumber("Auto Score Aim Height", 0);
    // double nAutoScoreSpeed = SmartDashboard.getNumber("Auto Score Speed", 0);
    // double nAutoScoreSpeedMin = SmartDashboard.getNumber("Auto Score Speed Min", 0);
    // double nAutoScoreSpeedMax = SmartDashboard.getNumber("Auto Score Speed Max", 0);

    // if (nAmpScoreAngle != LauncherConstants.AmpScoreAngle) {LauncherConstants.AmpScoreAngle = nAmpScoreAngle;}
    // if (nAmpScoreSpeed != LauncherConstants.AmpScoreSpeed) {LauncherConstants.AmpScoreSpeed = nAmpScoreSpeed;}
    // if (nSpeakerScoreAngle != LauncherConstants.SpeakerScoreAngle) {LauncherConstants.SpeakerScoreAngle = nSpeakerScoreAngle;}
    // if (nSpeakerScoreSpeed != LauncherConstants.SpeakerScoreSpeed) {LauncherConstants.SpeakerScoreSpeed = nSpeakerScoreSpeed;}
    // if (nTrapScoreAngle != LauncherConstants.TrapScoreAngle) {LauncherConstants.TrapScoreAngle = nTrapScoreAngle;}
    // if (nTrapScoreSpeed != LauncherConstants.TrapScoreSpeed) {LauncherConstants.TrapScoreSpeed = nTrapScoreSpeed;}
    // if (nAutoScoreAimHeight != LauncherConstants.kAutoScoreAimHeight) {LauncherConstants.kAutoScoreAimHeight = nAutoScoreAimHeight;}
    // if (nAutoScoreSpeed != LauncherConstants.kAutoScoreSpeed) {LauncherConstants.kAutoScoreSpeed = nAutoScoreSpeed;}
    // if (nAutoScoreSpeedMin != LauncherConstants.kAutoScoreSpeedMin) {LauncherConstants.kAutoScoreSpeedMin = nAutoScoreSpeedMin;}
    // if (nAutoScoreSpeedMax != LauncherConstants.kAutoScoreSpeedMax) {LauncherConstants.kAutoScoreSpeedMax = nAutoScoreSpeedMax;}
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
    RobotContainer.driverXbox.setRumble(RumbleType.kBothRumble, 0);
    LEDsSubSystem.setLED(.99);
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    aprilTagAlliance();
    // new ClimberInitCmd(climberSubsystem);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    aprilTagAlliance();
    RobotContainer.driverXbox.setRumble(RumbleType.kBothRumble, 0);
    //m_robotContainer.setMotorBrake(true);
    //new ClimberInitCmd(climberSubsystem);
    new ClimberInitCmd(climberSubsystem);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.spencerButtons();
    watchForNote();

    //System.out.println(sensorOuttake.get());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
        try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
  
    public static void aprilTagAlliance(){
    Optional<Alliance> allianceColor = DriverStation.getAlliance();
    if (allianceColor.isPresent()) {
      if (allianceColor.get() == Alliance.Red) {
        AprilTagConstants.ampID     = 5;
        AprilTagConstants.speakerID = 4;
        AprilTagConstants.stageIDA  = 13;
        AprilTagConstants.stageIDB  = 12;
        AprilTagConstants.stageIDC  = 11;
      }
      if (allianceColor.get() == Alliance.Blue) {
        AprilTagConstants.ampID     = 6;
        AprilTagConstants.speakerID = 7;
        AprilTagConstants.stageIDA  = 14;
        AprilTagConstants.stageIDB  = 15;
        AprilTagConstants.stageIDC  = 16; // Matts Awesome Please....
      }
     }
  }

  public static boolean watchForNote(){
    boolean hasTargets = false;
    if (sensorIntake.get() == false && sensorOuttake.get() == false){
      var result = camObj.getLatestResult(); //Get the latest result from PhotonVision
      hasTargets = result.hasTargets(); // Check if the latest result has any targets.
      if (hasTargets == true){
        //System.out.println("Note Found - Press and hold B to retrieve the note!");
       // LEDsSubSystem.setLEDwBlink(.65, .125); Matt thinks this is annoying (Blinking = Annoying)
        LEDsSubSystem.setLED(.25); //From .23
        //RobotContainer.pulseRumble();
      } else{
        //RobotContainer.driverXbox.setRumble(RumbleType.kBothRumble, 0);
        LEDsSubSystem.setLED(.99);
      }
    }
    return hasTargets;
  }
}
