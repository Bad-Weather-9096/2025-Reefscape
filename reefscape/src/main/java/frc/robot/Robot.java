// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Robot extends TimedRobot {
  private SparkMax sparkMax = new SparkMax(1, MotorType.kBrushless);

  private XboxController xboxController = new XboxController(0);

  public Robot() {
    var globalConfig = new SparkMaxConfig();

    globalConfig.smartCurrentLimit(50);

    sparkMax.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Output", sparkMax.getAppliedOutput());
  }
  
  @Override
  public void teleopPeriodic() {
    sparkMax.set(xboxController.getLeftY());
  }
}
