// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class moveDatArm extends PIDCommand {
  /** Creates a new moveDatArm. */
  public moveDatArm(Arm arm, double position) {
    super(
        // The controller that the command will use
        new PIDController(0.9, 0.6, 0.0),
        // This should return the measurement
        () -> arm.getArmPosition(),
        // This should return the setpoint (can also be a constant)
        () -> position,
        // This uses the output
        output -> {
          // Use the output here
          arm.setArmMotorSpeed(output + 0.1, output + 0.1);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.03);

    if (arm.getarmlimit1()){
      arm.StopArmMotors();
    }

    if(arm.getArmlimit2()){
      arm.StopArmMotors();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
