// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot_MM;

public class Pivot_To_Setpoint extends CommandBase {
  private final Pivot_MM m_Pivot;
  private double m_setpoint;
 
  /** Creates a new Pivot_To_Setpoint. */
  public Pivot_To_Setpoint(double deg, Pivot_MM subsystem) {
    m_setpoint = deg;
    m_Pivot = subsystem;
    addRequirements(m_Pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Pivot.my_motionMagic_Run(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
      
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Pivot.my_get_PositionLock(m_setpoint);
  }


 

}
