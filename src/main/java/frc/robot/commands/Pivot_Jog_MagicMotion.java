// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot_MM;

public class Pivot_Jog_MagicMotion extends CommandBase {
  private final Pivot_MM m_Pivot;
  private double m_setpoint;
  private double power;
  /** Creates a new Pivot_Jog_MagicMotion. */
  public Pivot_Jog_MagicMotion(Pivot_MM subsystem, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.power = power;
    m_Pivot = subsystem;
    addRequirements(m_Pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double maxStep = 405; //~1 degree //910;  //[counts} ~2 degrees]
    m_setpoint = (m_Pivot.get_My_CurrentRAW_Position() + (power * maxStep));  
    m_Pivot.my_motionMagic_Run(m_setpoint);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Pivot.my_motionMagic_Run(m_Pivot.my_getCurrentPivotAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
