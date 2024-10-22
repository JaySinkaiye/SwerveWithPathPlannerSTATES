// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private TalonFX LeftIntakeMotor = new TalonFX(53);
  private TalonFX RightIntakeMotor = new TalonFX(54);
  
  //use the WPI version like
  private WPI_VictorSPX LowerIntakeMotor = new WPI_VictorSPX(50);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  private final Color kOrangeTarget = new Color(0.643066, 0.314453, 0.042969);


  public Intake() {

    RightIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    LeftIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.addColorMatch(kOrangeTarget);    

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else if (match.color == kOrangeTarget) {
      colorString = "Orange";
    } else {
      colorString = "Unknown";
    }
   

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    //SmartDashboard.putNumber("Orange", detectedColor.Orange);


    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }

  public Command setIntakeMotorSpeed(double iSpeed, double lSpeed, double rSpeed){

    return startEnd(() ->{

    LeftIntakeMotor.set(lSpeed);
    RightIntakeMotor.set(rSpeed);
    LowerIntakeMotor.set(ControlMode.PercentOutput, iSpeed); 
    }, () ->{

    LeftIntakeMotor.set(0);
    RightIntakeMotor.set(0);
    LowerIntakeMotor.set(ControlMode.PercentOutput, 0);
    }
    ); 
  }

  public void SuckErIn(CommandXboxController trigger, CommandXboxController otherTrigger){
    LowerIntakeMotor.set(-0.8 * trigger.getLeftTriggerAxis());

    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kOrangeTarget){
        LowerIntakeMotor.set(0);
    } else if(match.color == kOrangeTarget && otherTrigger.povLeft().getAsBoolean()){
      LowerIntakeMotor.set(-0.8);
    }
  }

  public void ShootErOut(){
    LeftIntakeMotor.set(0.3);
    RightIntakeMotor.set(0.3);
    //LowerIntakeMotor.set(ControlMode.PercentOutput, -0.75 * trigger.getRightTriggerAxis());

  }

  public Command trapShoot(){
    return run(
      () -> {
        LeftIntakeMotor.set(0.2);
        RightIntakeMotor.set(0.35);
      }
    );
  }

  public void TRAPshoot(){
    LeftIntakeMotor.set(.2);
    RightIntakeMotor.set(0.4);
  }

  public void SpitErOut(){
    LowerIntakeMotor.set(ControlMode.PercentOutput, .8);
  }

  public void CoughErOut(){
    LeftIntakeMotor.set(-.8);
    RightIntakeMotor.set(-.8);
    LowerIntakeMotor.set(ControlMode.PercentOutput, 1);
  }

  public void StopArmMotor(){
    LeftIntakeMotor.set(0);
    RightIntakeMotor.set(0);
    LowerIntakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void stopMuzzleMotor(){
    LeftIntakeMotor.set(0);
    RightIntakeMotor.set(0);
  }

}
