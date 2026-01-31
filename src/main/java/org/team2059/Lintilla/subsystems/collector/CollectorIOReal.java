// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla.subsystems.collector;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectorIOReal implements CollectorIO {
  
  private SparkFlex tiltMotor;
  private SparkFlex intakeMotor;

  public CollectorIOReal() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
