package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

/** End effector rollers for coral and algae, with a break beam for detecting algae. */
public class GripperSubsystem extends SubsystemBase {
  private final SparkMax gripperMotor = new SparkMax(GripperConstants.kGripperMotorID, MotorType.kBrushless);
  private final DigitalInput breakBeam = new DigitalInput(GripperConstants.kBreakBeamID);

  /** The break beam only signals an algae pickup while we are actively intaking algae. */
  private boolean intakingAlgae = false;

  public GripperSubsystem() {
    configureGripperMotor();
  }

  public Command runGripper(double power) {
    return runOnce(() -> gripperMotor.set(power));
  }

  /**
   * Sets whether the robot is intaking algae. Does not require the subsystem, so it can be used
   * without interrupting a running gripper command.
   */
  public Command setIntakingAlgae(boolean intaking) {
    return Commands.runOnce(() -> intakingAlgae = intaking);
  }

  public boolean isBreakBeamBroken() {
    return !breakBeam.get();
  }

  /** True when an algae has been pulled into the gripper. */
  public boolean hasAlgae() {
    return intakingAlgae && isBreakBeamBroken();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Gripper/Break Beam", isBreakBeamBroken());
    SmartDashboard.putBoolean("Gripper/Intaking Algae", intakingAlgae);
  }

  private void configureGripperMotor() {
    var config = new SparkMaxConfig();
    config
        .inverted(true)
        .smartCurrentLimit(GripperConstants.kSmartCurrentLimit)
        .idleMode(IdleMode.kBrake);

    gripperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
