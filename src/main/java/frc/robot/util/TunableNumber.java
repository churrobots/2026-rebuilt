// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class TunableNumber {

  private double m_initialValue;
  private String m_smartDashboardKey;

  public TunableNumber(String smartDashboardKey, double initialValue) {
    m_initialValue = initialValue;
    m_smartDashboardKey = smartDashboardKey;
    SmartDashboard.putNumber(m_smartDashboardKey, m_initialValue);
  }

  public double getLatest() {
    return SmartDashboard.getNumber(m_smartDashboardKey, m_initialValue);
  }
}
