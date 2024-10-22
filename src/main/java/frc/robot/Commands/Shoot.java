// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Intake;

public class Shoot extends Command {
  /** Creates a new Shoot. */

  private final Intake intake;
  private RobotContainer robotContainer;

  public Shoot(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotContainer = RobotContainer.getInstance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.SuckErIn(robotContainer.getOperatorJoystick(), robotContainer.getOperatorJoystick());
  
    if ((robotContainer.getOperatorJoystick().getRightTriggerAxis() >= 0.01)){
        intake.ShootErOut();
      } else if (robotContainer.getOperatorJoystick().getLeftY() >= 0.1  ){
        intake.TRAPshoot();
      }
      else {
        intake.stopMuzzleMotor();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(robotContainer.getOperatorJoystick().getRightTriggerAxis() >= 0.01){
      return false;
    } else {
    return true;
  }
  }
}
