// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  private static SparkMax elevatorLead = new SparkMax(ElevatorConstants.elevatorFID, MotorType.kBrushless);
  private static SparkMax elevatorFollow = new SparkMax(ElevatorConstants.elevatorLID, MotorType.kBrushless);
  private static final AbsoluteEncoder elevatorEncoder = elevatorLead.getAbsoluteEncoder();

  // 
  public ElevatorSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static double getPosition(){
    return elevatorEncoder.getPosition();
  }

  public void setElevatorPos(int posValue){
    if (posValue == 0){
      if (ElevatorSubsystem.getPosition() < ElevatorConstants.elevatorPos1){
        elevatorLead.set(.25);
        elevatorFollow.set(.25);
      } else if (ElevatorSubsystem.getPosition() == 0){
        elevatorLead.set(0);
        elevatorFollow.set(0);
      }
    } else if (posValue == 1){
      if (ElevatorSubsystem.getPosition() < ElevatorConstants.elevatorPos2){
        elevatorLead.set(.25);
        elevatorFollow.set(.25);
      } else if (ElevatorSubsystem.getPosition() == 1){
        elevatorLead.set(0);
        elevatorFollow.set(0);
      }
    } else if (posValue == 2){
      if (ElevatorSubsystem.getPosition() < ElevatorConstants.elevatorPos3){
        elevatorLead.set(.25);
        elevatorFollow.set(.25);
      } else if (ElevatorSubsystem.getPosition() == 2){
        elevatorLead.set(0);
        elevatorFollow.set(0);
      }
    }
  }
}
