// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm_MM;

public class Arm_Jog_MagicMotion extends CommandBase {
  private final Arm_MM m_Arm;
  private double m_setpoint;
  private double power;
  /** Creates a new Arm_Jog_MagicMotion. */
  public Arm_Jog_MagicMotion(Arm_MM subsystem, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.power = power;
    m_Arm = subsystem;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double maxStep = 2; //inches
    m_setpoint = (m_Arm.my_getCurrentExtensionLength() + (power * maxStep));  //add tp subsystm
    m_Arm.my_motionMagic_Run(m_setpoint);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.my_motionMagic_Run(m_Arm.my_getCurrentExtensionLength());  //create in subsystem
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
