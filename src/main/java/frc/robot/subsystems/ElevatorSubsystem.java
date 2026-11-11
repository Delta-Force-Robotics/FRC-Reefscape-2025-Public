package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import java.util.function.DoubleSupplier;

/** Two-stage elevator driven by two Krakens, the back motor following the front one. */
public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX frontMotor = new TalonFX(ElevatorConstants.kFrontMotorID, Constants.kCanBus);
  private final TalonFX backMotor = new TalonFX(ElevatorConstants.kBackMotorID, Constants.kCanBus);

  private final MotionMagicDutyCycle motionMagicRequest =
      new MotionMagicDutyCycle(0).withEnableFOC(true).withSlot(0);

  public ElevatorSubsystem() {
    configureMotors();
    frontMotor.setControl(motionMagicRequest.withPosition(ElevatorConstants.kInterPos));
  }

  /** Moves the elevator to a fixed position. */
  public Command moveElevator(double position) {
    return moveElevator(() -> position);
  }

  /** Moves the elevator to a position evaluated when the command runs. */
  public Command moveElevator(DoubleSupplier position) {
    return runOnce(() -> frontMotor.setControl(motionMagicRequest.withPosition(position.getAsDouble())));
  }

  public double getPosition() {
    return frontMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/Position", getPosition());
  }

  private void configureMotors() {
    var config = new TalonFXConfiguration();

    config.Slot0.kP = ElevatorConstants.kP;
    config.Slot0.kI = ElevatorConstants.kI;
    config.Slot0.kD = ElevatorConstants.kD;
    config.Slot0.kG = ElevatorConstants.kG;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kAcceleration;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.kSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // The follower ignores its own inversion setting, but still needs the brake mode and current
    // limits, so the same config is applied to both motors.
    frontMotor.getConfigurator().apply(config);
    backMotor.getConfigurator().apply(config);
    backMotor.setControl(new Follower(frontMotor.getDeviceID(), true));

    // The elevator starts every match fully down.
    frontMotor.setPosition(0);
    backMotor.setPosition(0);
  }
}
