// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot_MM;

public class Pivot_Percent extends CommandBase {
  private final Pivot_MM m_Pivot;
  private double m_setpoint;
  /** Creates a new Pivot_Percent. */
  public Pivot_Percent(double setpoint, Pivot_MM subsystem) {
    m_setpoint = setpoint;
    m_Pivot = subsystem;
    addRequirements(m_Pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Pivot.my_PercentOutput_Run(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Pivot.my_PercentOutput_Run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
