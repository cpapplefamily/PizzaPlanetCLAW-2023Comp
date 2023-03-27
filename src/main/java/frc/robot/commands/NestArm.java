// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm_MM;
import frc.robot.subsystems.Pivot_MM;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NestArm extends SequentialCommandGroup {
  /** Creates a new NestArm. */
  public NestArm(double pivotPosition, double armPosition, Pivot_MM m_pivot_MM, Arm_MM m_arm_MM) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Arm_To_Setpoint(armPosition, m_arm_MM),
      new WaitCommand(0.5),
      new Pivot_To_Setpoint(pivotPosition, m_pivot_MM),
      new WaitCommand(0.5)
    );
  }
}
