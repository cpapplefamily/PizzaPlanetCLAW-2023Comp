// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm_MM;

public class Arm_Percent extends CommandBase {
  private final Arm_MM m_Arm;
  private double m_setpoint;


  /** Creates a new Arm_Percent. */
  public Arm_Percent(double setpoint, Arm_MM subsystem) {
    m_setpoint = setpoint;
    m_Arm = subsystem;
    addRequirements(m_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.my_PercentOutput_Run(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.my_PercentOutput_Run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Arm.DisableRetractMotion();
    //Check for extension limit??
    //return false;
  }

}