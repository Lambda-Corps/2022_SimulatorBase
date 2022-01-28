// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import static frc.robot.Constants.*;

public class DriveManuallyCommand extends CommandBase {
  /** Creates a new DriveManuallyCommand. */

  // Reference to the constructed drive train from RobotContainer to be
  // used to drive our robot
  private final DriveTrainSubsystem m_driveTrain;
  private final XboxController m_driverController;

  public DriveManuallyCommand(DriveTrainSubsystem driveTrain, XboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_driverController = driverController;

    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Axis are inverted, negate them so positive is forward
    double turn = m_driverController.getRawAxis(DRIVER_RIGHT_AXIS); // Right X
    double move = -m_driverController.getRawAxis(DRIVER_LEFT_AXIS); // Left Y

    m_driveTrain.manualDrive(move, turn);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
