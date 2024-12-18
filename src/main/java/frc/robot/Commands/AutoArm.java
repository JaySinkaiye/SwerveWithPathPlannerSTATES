// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Arm;

public class AutoArm extends Command {
  /** Creates a new AutoArm. */

  private final Arm arm;
  private RobotContainer robotContainer;
  double desiredSpeed;

  public AutoArm(Arm arm, double desiredSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.desiredSpeed = desiredSpeed;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotContainer = RobotContainer.getInstance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmMotorSpeed(desiredSpeed, desiredSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getArmlimit2();
  }
}
