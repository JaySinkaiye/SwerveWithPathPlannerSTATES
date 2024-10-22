// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ArmUp;
import frc.robot.Commands.AutoArm;
import frc.robot.Commands.Climb;
import frc.robot.Commands.Rotate;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Commands.moveDatArm;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.agenerated.TunerConstants;

public class RobotContainer {
  public static RobotContainer robotContainer = new RobotContainer();

  //subsystems
  public final Climber climber = new Climber();
  public final Arm arm = new Arm();
  public final Intake intake = new Intake();

  //private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  //private double MaxAngularRate = 1.1 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController DriverJoystick = new CommandXboxController(0); // My DriverJoystick
  private final CommandXboxController OperatorJoystick = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  

  private final SendableChooser<Command> ManualChooser = new SendableChooser<>();

  private RobotContainer() {

    //register named commands
    NamedCommands.registerCommand("Intake", intake.setIntakeMotorSpeed(-0.6,0,0));
    NamedCommands.registerCommand("RevUp", intake.setIntakeMotorSpeed(0, 0.6, 0.6));
    NamedCommands.registerCommand("RevUp2", intake.setIntakeMotorSpeed(0, 0.5, 0.5));
    NamedCommands.registerCommand("Shoot", intake.setIntakeMotorSpeed(-0.6, 0.7, 0.7));
    NamedCommands.registerCommand("Shoot2", intake.setIntakeMotorSpeed(-0.6, 0.3, 0.3));
    NamedCommands.registerCommand("RevUp3", intake.setIntakeMotorSpeed(0, 0.4, 0.4));
    NamedCommands.registerCommand("Shoot3", intake.setIntakeMotorSpeed(-0.6, 0.3, 0.3));
    NamedCommands.registerCommand("NewArmDown", new AutoArm(arm, -0.3));
    NamedCommands.registerCommand("FastArmDown", new AutoArm(arm, -0.6));
    NamedCommands.registerCommand("ArmToAmp", new moveDatArm(arm, 0.01));
    NamedCommands.registerCommand("ArmToHuman", new moveDatArm(arm, 0.2));
    NamedCommands.registerCommand("ArmToLimit", new AutoArm(arm, -0.3));
    NamedCommands.registerCommand("RaiseToShoot", new moveDatArm(arm, 0.1));
    NamedCommands.registerCommand("RaiseToAmp", new ArmUp(arm));

    configureBindings();

    //auto chooser

    SmartDashboard.putData("Manual Chooser", ManualChooser);
    ManualChooser.setDefaultOption("States center 2", new PathPlannerAuto("State center 2"));
    ManualChooser.addOption("States center 4(amp side first)", new PathPlannerAuto("State center 4 amp first"));
    ManualChooser.addOption("States center 4 (Source side first)", new PathPlannerAuto("State center 4 source first"));

    ManualChooser.addOption("States Speaker score Source backout", new PathPlannerAuto("State Speaker Score Source backout"));
    ManualChooser.addOption("States Speaker Score Amp backout", new PathPlannerAuto("State Speaker score Amp backout"));

    ManualChooser.addOption("States Amp Score 1", new PathPlannerAuto("State Amp Score 1"));
    ManualChooser.addOption("States Amp Score 2", new PathPlannerAuto("State Amp Score 2"));
    ManualChooser.addOption("States Amp score(1) then center pickup", new PathPlannerAuto("State Amp Score Center pickup"));
    
    // end of auto choose

    climber.setDefaultCommand(new Climb(climber));
    arm.setDefaultCommand(new Rotate(arm, ()-> OperatorJoystick.getRightY()));
    intake.setDefaultCommand(new Shoot(intake));
  }

  public static RobotContainer getInstance(){
      return robotContainer;
    }
  
  private void configureBindings() {
    drivetrain.setDefaultCommand(
          new SwerveDrive(
            drivetrain, 
            () -> -DriverJoystick.getLeftX(),  //Translation 
            () -> -DriverJoystick.getLeftY(),  //Translation
            () -> -DriverJoystick.getRightX(), //Rotation
            DriverJoystick.povUp(), 
            DriverJoystick.povDown(), 
            DriverJoystick.y(), //Face Forward
            DriverJoystick.b(), //Face Right
            DriverJoystick.a(), //Face Backwards
            DriverJoystick.x()  //Face Left
          )
        );

    // reset the field-centric heading on left bumper press
    DriverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    DriverJoystick.rightBumper().onTrue(drivetrain.runOnce(()-> drivetrain.seedFieldRelative()));

    // spit out game piece if stuck
    OperatorJoystick.rightBumper().whileTrue(intake.run(() -> intake.CoughErOut()));

    // Arm Setpoints 
    OperatorJoystick.x().whileTrue(new moveDatArm(arm, 0.1).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    OperatorJoystick.y().whileTrue(new moveDatArm(arm, 0.4).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    //OperatorJoystick.b().onTrue(new moveDatArm(arm, 0.2).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    //OperatorJoystick.x().onTrue(arm.setAAngle(.2).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    //OperatorJoystick.y().onTrue(new crankDatArm(arm, 0.01).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    //OperatorJoystick.b().onTrue(new crankDatArm(arm, 0.2).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }

  public CommandXboxController getOperatorJoystick(){
    return OperatorJoystick;
  }

  public CommandXboxController getDriverJoystick(){
    return DriverJoystick;
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return ManualChooser.getSelected();
  }
}
