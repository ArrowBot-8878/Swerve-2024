package frc.robot.Commands.AutoCommands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.FeederLockOn;
import frc.robot.Commands.ArmControl.ClosedLoopArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFeedAlign extends ParallelCommandGroup {
  /** Creates a new AutoAimWithShooterAngle. */
  public AutoFeedAlign(DriveSubsystem m_DriveSubsystem, Arm m_Arm, Shooter m_Shooter, DoubleSupplier xDriveSupplier, DoubleSupplier yDriveSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FeederLockOn(m_DriveSubsystem, xDriveSupplier, yDriveSupplier),
    // new ShooterWristClosedLoopTracking(m_ShooterWrist, ()->AutoShoot.FEEDER_DISTANCE_TO_ANGLE_MAP.get(PhotonVision.getDistanceToFeeder(m_DriveSubsystem.getPose2d()))));
    new ClosedLoopArm(m_Arm, 65.0), 
    new ShooterFire(m_Shooter, ()-> 0.3));
  }
}

