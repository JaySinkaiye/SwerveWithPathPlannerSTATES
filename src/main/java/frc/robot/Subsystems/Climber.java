// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.hardware.TalonFX;

public class Climber extends SubsystemBase {

  //private CANSparkMax ClimbMotor = new CANSparkMax(60, MotorType.kBrushless);
  private TalonFX ClimbMotor = new TalonFX(60);


  //private final double kClimbTick2Deg = 360.0 / 2048 *20;`

  /** Creates a new Climber. */
  public Climber() {
    ClimbMotor.setPosition(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb encoder", getClimbPosition());
  }

  public void setMotorSpeed(double speed){
    ClimbMotor.set(speed);
  }

  public void Climb(CommandXboxController stick){
    if (getClimbPosition() <= 0){
        if (stick.getRightTriggerAxis() > 0.05){
        ClimbMotor.set(-0.85 * stick.getRightTriggerAxis());
      } else{
        ClimbMotor.set(0);
      }
    } else if (getClimbPosition() >= 224){
        if (stick.getRightTriggerAxis() > 0.05){
        ClimbMotor.set(0);
      } else{
        ClimbMotor.set(0.85 * stick.getLeftTriggerAxis());
      }
    } else {
        if (stick.getRightTriggerAxis() > 0.05){
        ClimbMotor.set(-0.85 * stick.getRightTriggerAxis());
      } else{
        ClimbMotor.set(.85 * stick.getLeftTriggerAxis());
      }
    }
  }

  public void StopClimbMotor(){
    ClimbMotor.set(0);
  }

  public double getClimbPosition(){
    return ClimbMotor.getPosition().getValueAsDouble() * -1;
  }

}
