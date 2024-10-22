// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;

public class crankDatArm extends Command {
  /** Creates a new crankDatArm. */

  private final Arm arm;
  private double pos;

  public crankDatArm(Arm arm, double desiredPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    pos = desiredPosition;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setAngle(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (arm.isWithinPosTolerance(pos)){
      return true;
    }
    return false;
  }
}
