// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Arm;

public class Rotate extends Command {
  /** Creates a new Rotate. */

  private final Arm arm;
  private RobotContainer robotContainer;

  private DoubleSupplier ySup;
  private double yVal;

  
  public Rotate(Arm arm, DoubleSupplier ySup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.ySup = ySup;
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
    yVal = MathUtil.applyDeadband(ySup.getAsDouble(), 0.1);
    arm.RotateArm(yVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
