// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Flywheel;

/**
 * This is a sample program to demonstrate how to use a state-space controller to control a
 * flywheel.
 */
public class Robot extends TimedRobot {

  private final boolean runSysId = true;

  private static final int kControllerPort = 0;
  private static final double kSpinupRadPerSec = 60;
  
  private final PS5Controller m_joystick = new PS5Controller(kControllerPort);

  private Flywheel m_flywheel = new Flywheel(0);

  @Override
  public void robotInit() {
    SmartDashboard.putData("Run System Identification", new ProxyCommand(this::makeSysIdCommand));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    if (runSysId) {
      return;
    }
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

  private Command makeSysIdCommand() {
    return new SequentialCommandGroup(
      new PrintCommand("Beginning System Identification"),
      m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward),
      m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse),
      m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
      m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
      new PrintCommand("Finished System Identification")
    );
  }
}
