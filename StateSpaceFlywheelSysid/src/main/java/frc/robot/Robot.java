// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.subsystems.Flywheel;

/**
 * This is a sample program to demonstrate how to use a state-space controller to control a
 * flywheel.
 */
public class Robot extends TimedRobot {

  private static final int kControllerPort = 0;
  private static final double kSpinupRadPerSec = 60;
  // A joystick to read the trigger from.
  private final PS5Controller m_joystick = new PS5Controller(kControllerPort);

  private Flywheel m_flywheel = new Flywheel(0);

  @Override
  public void robotInit() {
  }

  @Override
  public void teleopPeriodic() {
    // Sets the target speed of our flywheel. This is similar to setting the setpoint of a
    // PID controller.
    if (m_joystick.getR1ButtonPressed()) {
      // We just pressed the trigger, so let's set our next reference
      m_flywheel.setVelocity(kSpinupRadPerSec);
    } else if (m_joystick.getR1ButtonReleased()) {
      // We just released the trigger, so let's spin down
      m_flywheel.setVelocity(0.0);
    }

  }
}
