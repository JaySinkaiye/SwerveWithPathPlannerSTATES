// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private TalonFX LeftArmMotor = new TalonFX(51);
  private TalonFX RightArmMotor = new TalonFX(52);
  //private TalonFX armMotor = new TalonFX(55);
  private CANcoder ArmCancoder = new CANcoder(2);

  public final DigitalInput limitSwitch1 = new DigitalInput(0);
  public final DigitalInput limitSwitch2 = new DigitalInput(1);

  public Arm() {
    applyArmMotorConfigs(InvertedValue.Clockwise_Positive);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Absolute Encoder value", ArmCancoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.getBoolean("Limit1", getarmlimit1());
    SmartDashboard.putBoolean("Limit2", getArmlimit2());
    SmartDashboard.putNumber("RightVel", RightArmMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("LeftVel", LeftArmMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Arm in degrees", getAngleDegrees());
    SmartDashboard.putNumber("Distance from Limelight", LimelightHelpers.getDistance());

    if (limitSwitch1.get()){
      setArmPosition(0.98);
      StopArmMotors();
    }

    if (limitSwitch2.get()){
      setArmPosition(0.03247);
      StopArmMotors();
    }

  }

  public void setArmMotorSpeed(double lspeed, double rspeed){
    LeftArmMotor.set(lspeed);
    RightArmMotor.set(rspeed);
    //armMotor.set(speed);
  }

  public void RotateArm (double yVal){
    if(getarmlimit1()){
      if(yVal >= 0){
        LeftArmMotor.set(0);
        RightArmMotor.set(0);
        //armMotor.set(0);
      } else {
        LeftArmMotor.set(0.5 * yVal);
        RightArmMotor.set(0.5 * yVal);
        //armMotor.set(0);
      }
    } else if(getArmlimit2()){
      if(yVal <= 0){
        LeftArmMotor.set(0);
        RightArmMotor.set(0);
        //armMotor.set(0);
      } else {
        LeftArmMotor.set(0.5 * yVal);
        RightArmMotor.set(0.5 * yVal);
        //armMotor.set(0.1 * yVal);
      }
    } else {
    LeftArmMotor.set(0.5 * yVal);
    RightArmMotor.set(0.5 * yVal);
    //armMotor.set(0.1 * yVal);
    }

  }

  public void StopArmMotors(){
    LeftArmMotor.set(0);
    RightArmMotor.set(0);
    //armMotor.set(0);
  }

  public double getArmPosition(){
    return ArmCancoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getAngleDegrees(){
    return (getArmPosition()*90)+2;
  }

  public void setArmPosition(double position){
    ArmCancoder.setPosition(position);
  }

 public boolean getarmlimit1(){
    return limitSwitch1.get();
  }

  public boolean getArmlimit2(){
    return limitSwitch2.get();
  }

  // motion magic
  public void setAngle(double pos){
    final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(pos);
    RightArmMotor.setControl(request);
    LeftArmMotor.setControl(request);
  }

  /*public Command setAAngle(double pos){
    final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(pos);
    return startEnd(()->{
      RightArmMotor.setControl(request);
      LeftArmMotor.setControl(request);
    }, ()->{
      RightArmMotor.set(0);
      LeftArmMotor.set(0);
    });
    
  }
  */

  public Command gotToPos(double pos){
    final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(pos);
    return runOnce(
      ()->{
        RightArmMotor.setControl(request);
        LeftArmMotor.setControl(request);
      }
    );
  }

  private void applyArmMotorConfigs(InvertedValue inversion){
    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    talonConfigs.Slot0.kP = 10;
    talonConfigs.Slot0.kI = 0;
    talonConfigs.Slot0.kD = 0;
    talonConfigs.Slot0.kV = 0;
    talonConfigs.Slot0.kG = 0.29;
    talonConfigs.Slot0.kS = 0;
    talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    var motionMagicConfigs = talonConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 5;
    motionMagicConfigs.MotionMagicAcceleration = 10;
    motionMagicConfigs.MotionMagicJerk = 30;

    talonConfigs.Feedback.FeedbackRemoteSensorID = ArmCancoder.getDeviceID();
    talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    LeftArmMotor.getConfigurator().apply(talonConfigs);
    RightArmMotor.getConfigurator().apply(talonConfigs);

    MotorOutputConfigs motorOutputConfigsRight = new MotorOutputConfigs();
    motorOutputConfigsRight.Inverted = inversion;
    motorOutputConfigsRight.NeutralMode = NeutralModeValue.Brake;
    MotorOutputConfigs motorOutputConfigsLeft = new MotorOutputConfigs();
    motorOutputConfigsLeft.NeutralMode = NeutralModeValue.Brake;
    RightArmMotor.getConfigurator().apply(motorOutputConfigsRight);
    LeftArmMotor.getConfigurator().apply(motorOutputConfigsLeft);

  }


  public boolean isWithinPosTolerance(double pos){
    return((getArmPosition() >= pos - 0.05) && (getArmPosition() <= pos + 0.05));
  }
  
}
