// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {


  private final SparkMax liftingMotor1;
  private final SparkMax liftingMotor2;


  private final RelativeEncoder l1_Encoder;
  private final RelativeEncoder l2_Encoder;


  private final SparkClosedLoopController l1Motor_ClosedLoopContoller;
  private final SparkClosedLoopController l2Motor_ClosedLoopContoller;


  /** Creates a new Elevator. */
  public Elevator(int liftingMotor1ID, int liftingMotor2ID) {
    liftingMotor1 = new SparkMax(liftingMotor2ID, MotorType.kBrushless);
    liftingMotor2 = new SparkMax(liftingMotor2ID, MotorType.kBrushless);


    l1_Encoder = liftingMotor1.getEncoder();
    l2_Encoder = liftingMotor2.getEncoder();


    l1Motor_ClosedLoopContoller = liftingMotor1.getClosedLoopController();
    l2Motor_ClosedLoopContoller = liftingMotor2.getClosedLoopController();

    l1_Encoder.setPosition(0);
    l2_Encoder.setPosition(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}