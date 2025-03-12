// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

//import getFPGATime
import edu.wpi.first.wpilibj.RobotController;

public class FunnelSubsystem extends SubsystemBase {
  /** Creates a new FunnelSubsystem. */
  
  private static SparkMax funnelMotor = new SparkMax(3, MotorType.kBrushless);

  public FunnelSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void funnelUp(){
    long millisecondsToRun = 1000;
    long initTime = RobotController.getFPGATime();
    funnelMotor.set(-0.1);
  }
  
  public void funnelStop(){
    funnelMotor.set(0);
  }

  public void funnelDown(){
    funnelMotor.set(0.1);
  }
} 
