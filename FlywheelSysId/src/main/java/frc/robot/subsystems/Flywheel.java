// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.MutableMeasure.mutable;

import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Flywheel extends SubsystemBase {
  private final int k_flywheelPort = 0;

  private TalonFX m_flywheel = new TalonFX(k_flywheelPort);

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      (Measure<Voltage> volts) -> {
        m_flywheel.setVoltage(volts.in(Volts));
      },
      log -> {
        log.motor("flywheel")
          .voltage(
            m_appliedVoltage.mut_replace(m_shooterMotor.get() * RobotController.getBatteryVoltage(), Volts))
          .angulatPosition(m_angle.mut_replace(m_shooterEncoder.getDistance(), Rotations))
          .angularVelocity(m_velocity.mut_replace(m_shooterEncoder.getRate(), RotationsPerSecond));
      },
      null
      )
  );


  /** Creates a new Flywheel. */
  public Flywheel() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
