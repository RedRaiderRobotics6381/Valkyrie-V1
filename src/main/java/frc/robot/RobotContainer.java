// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Secondary.ClimbCmd;
import frc.robot.commands.Secondary.ClimberInitCmd;
import frc.robot.commands.Secondary.IntakeCmd;
import frc.robot.commands.Secondary.LowerCmd;
import frc.robot.commands.Secondary.RVEIntakeCmd;
import frc.robot.commands.Secondary.SafeScoreCmd;
import frc.robot.commands.Secondary.ScoreAmpCmd;
import frc.robot.commands.Secondary.ScoreAutoCmd;
import frc.robot.commands.Secondary.ScoreSpeakerCmd;
//import frc.robot.commands.Secondary.ScoreTrapCmd;
import frc.robot.commands.Vision.DriveToAmpCmd;
import frc.robot.commands.Vision.DriveToSpeakerCmd;
import frc.robot.commands.Vision.DriveToStageCmd;
import frc.robot.commands.Vision.LauncherAimAutonCMD;
//import frc.robot.commands.Vision.LauncherAimCMD;
import frc.robot.commands.Vision.PickUpNoteCmd;
import frc.robot.subsystems.Secondary.ClimberSubsystem;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.Secondary.LauncherSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  public static XboxController driverXbox = new XboxController(0);
  public static XboxController engineerXbox = new XboxController(1);
  private final SendableChooser<Command> autoChooser;
  static double lastTime = -1;

  //LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  LauncherRotateSubsystem launcherRotateSubsystem = new LauncherRotateSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Register Named Commands

    //NamedCommands.registerCommand(null, null);
    // NamedCommands.registerCommand("Shoot", new ScoreAutoCmd(launcherSubsystem));
    //drivebase.setupPathPlanner();
    NamedCommands.registerCommand("Shoot", new ScoreAutoCmd(launcherSubsystem, launcherRotateSubsystem, intakeSubsystem));
    NamedCommands.registerCommand("Aim", new ScoreSpeakerCmd(launcherSubsystem, launcherRotateSubsystem, intakeSubsystem));
    // NamedCommands.registerCommand("PVAim", new LauncherAimCMD(launcherRotateSubsystem));
    NamedCommands.registerCommand("PVAim", new LauncherAimAutonCMD(launcherRotateSubsystem, launcherSubsystem, intakeSubsystem));
    NamedCommands.registerCommand("Intake", new IntakeCmd(intakeSubsystem, launcherRotateSubsystem));
    NamedCommands.registerCommand("DriveToNote", new PickUpNoteCmd(drivebase, intakeSubsystem, launcherRotateSubsystem));
    NamedCommands.registerCommand("DriveToSpeaker", new DriveToSpeakerCmd(drivebase));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) *
                                                            Constants.Drivebase.Max_Speed_Multiplier,
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) *
                                                            Constants.Drivebase.Max_Speed_Multiplier,
        () -> yawToSpeaker());

        Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
          () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) *
                                                               Constants.Drivebase.Max_Speed_Multiplier,
          () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) *
                                                               Constants.Drivebase.Max_Speed_Multiplier,
          () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(4), OperatorConstants.RIGHT_X_DEADBAND) *
                                                                       Constants.Drivebase.Max_Speed_Multiplier);

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    //Button 1 is "A" on xbox controller
    //Button 2 is "B" on xbox controller
    //Button 3 is "X" on xbox controller  
    //Button 4 is "Y" on xbox controller
    //Button 5 is "Left Bumper" on xbox controller
    //Button 6 is "Right Bumper" on xbox controller
    //Button 7 is "Back" on xbox controller
    //Button 8 is "Start" on xbox controller
    //Button 9 is "Left Joystick" on xbox controller
    //Button 10 is "Right Joystick" on xbox controller
    //Axis 0 is left joystick x side to side
    //Axis 1 is left joystick y forward and back
    //Axis 2 is left trigger 
    //Axis 3 is right trigger
    //Axis 4 is right joystick x side to side
    //Axis 5 is right joystick y forward and back

    // Buttons used in other parts of code! ====================================
    // Driver control button 2 is used in the pick up note command PickUpNoteCmd

    //==========================================================================
    new JoystickButton(driverXbox, 8).onTrue((new InstantCommand(drivebase::zeroGyro)));  //Button "Start"
    new JoystickButton(driverXbox, 1).whileTrue(new DriveToAmpCmd(drivebase)); 
    new JoystickButton(driverXbox, 2).whileTrue(new PickUpNoteCmd(drivebase, intakeSubsystem, launcherRotateSubsystem));  //Button "B"
    new JoystickButton(driverXbox, 3).whileTrue(new DriveToSpeakerCmd(drivebase)); //Button "X"
    new JoystickButton(driverXbox, 4).whileTrue(new DriveToStageCmd(drivebase)); //Button "Y"
    //Button 5 is used below in the spencerButtons method
    //Button 6 is used below in the spencerButtons method

    new JoystickButton(engineerXbox, 1).whileTrue(new LauncherAimAutonCMD(launcherRotateSubsystem, launcherSubsystem, intakeSubsystem)); //Button "A"
    //new JoystickButton(engineerXbox, 1).whileTrue(new LauncherAimCMD(launcherRotateSubsystem)); //Button "A"
    new JoystickButton(engineerXbox, 2).onTrue(new ScoreAmpCmd(launcherSubsystem, launcherRotateSubsystem, intakeSubsystem));
    new JoystickButton(engineerXbox, 3).onTrue(new ScoreSpeakerCmd(launcherSubsystem, launcherRotateSubsystem, intakeSubsystem));
    new JoystickButton(engineerXbox, 4).onTrue(new SafeScoreCmd(launcherSubsystem, launcherRotateSubsystem, intakeSubsystem));
    new JoystickButton(engineerXbox, 5).onTrue(new IntakeCmd(intakeSubsystem, launcherRotateSubsystem));
    new JoystickButton(engineerXbox, 6).onTrue(new ScoreAutoCmd(launcherSubsystem, launcherRotateSubsystem, intakeSubsystem));
    
    new POVButton(engineerXbox, 0).onTrue(new ClimbCmd(climberSubsystem));
    new POVButton(engineerXbox, 180).onTrue(new LowerCmd(climberSubsystem));
    new POVButton(engineerXbox, 90).whileTrue(new ClimberInitCmd(climberSubsystem));
    new POVButton(engineerXbox, 270).whileTrue(new RVEIntakeCmd(intakeSubsystem, launcherRotateSubsystem));
    new JoystickButton(driverXbox, 7).whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                              new Pose2d(new Translation2d(
                                  drivebase.getPose().getX() + 0.5 * Math.cos(drivebase.getPose().getRotation().getRadians()),
                                  drivebase.getPose().getY() + 0.5 * Math.sin(drivebase.getPose().getRotation().getRadians())),
                                  Rotation2d.fromDegrees(drivebase.getPose().getRotation().getDegrees())))));
                          
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void initClimber()
  {
    new ClimberInitCmd(climberSubsystem).schedule();
  }

  public void spencerButtons(){
    if (driverXbox.getRawButton(5) == true && driverXbox.getRawButton(6) == true){
      System.out.println("HighSpd");
      Constants.Drivebase.Max_Speed_Multiplier = 1;
    }
    if (driverXbox.getRawButton(5) == true && driverXbox.getRawButton(6) == false){
      System.out.println("MedSpd");
      Constants.Drivebase.Max_Speed_Multiplier = 0.75;
    }
    if (driverXbox.getRawButton(5) == false && driverXbox.getRawButton(6) == true){
      System.out.println("MedSpd");
      Constants.Drivebase.Max_Speed_Multiplier = 0.75;
    }

    if (driverXbox.getRawButton(5) == false && (driverXbox.getRawButton(6) == false)){
      //System.out.println("LowSpd");
      Constants.Drivebase.Max_Speed_Multiplier = 0.50;
    }
  }
      public double yawToSpeaker(){
      double yawToSpeakerValue = 0.0;
      PIDController zController = null;
      try {
        zController = new PIDController(.025, 0.05, 0.0);//0.07,0.0, 0.000;
        zController.setTolerance(.5);

        if (driverXbox.getRawButton(10) == false){  
          yawToSpeakerValue = MathUtil.applyDeadband(-driverXbox.getRawAxis(4), OperatorConstants.RIGHT_X_DEADBAND);
        } else{
          if (Robot.camAprTgLow.getLatestResult().hasTargets() == true){
            if (Robot.camAprTgLow.getLatestResult().getBestTarget().getFiducialId() == AprilTagConstants.speakerID){
              yawToSpeakerValue = MathUtil.clamp(zController.calculate(Robot.camAprTgLow.getLatestResult().getBestTarget().getYaw(),0), -1.0 , 1.0);
            }
          }
        }
      } finally {
        if (zController != null) {
          zController.close();
        }
      }
      return yawToSpeakerValue;
    }
}
