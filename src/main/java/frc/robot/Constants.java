// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Robot-wide constants: CAN IDs, control gains, mechanism setpoints and motor powers.
 *
 * <p>Positions are in mechanism rotations unless stated otherwise. Powers are duty cycle in the
 * range [-1, 1].
 */
public final class Constants {
  private Constants() {}

  /** CANivore bus shared by the swerve drive, elevator and arm. */
  public static final String kCanBus = "ctre";

  public static final class DriveConstants {
    private DriveConstants() {}

    /** Maximum angular velocity of the drivetrain, in rotations per second. */
    public static final double kMaxAngularRateRps = 0.75;

    /** Joystick deadband, as a fraction of max speed. */
    public static final double kDeadband = 0.1;

    /** Drive speed multiplier while the elevator is extended for scoring. */
    public static final double kScoringSpeedMultiplier = 0.3;

    /** Robot-relative speed (m/s) and duration (s) of the back-off move after scoring. */
    public static final double kBackOffSpeed = 1;
    public static final double kBackOffTime = 0.55;
  }

  public static final class ElevatorConstants {
    private ElevatorConstants() {}

    public static final int kFrontMotorID = 15;
    public static final int kBackMotorID = 14;

    // Motion Magic gains
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0.01;
    public static final double kG = 0;
    public static final double kCruiseVelocity = 400;
    public static final double kAcceleration = 400;
    public static final double kStatorCurrentLimit = 120;
    public static final double kSupplyCurrentLimit = 70;

    // Stowed and transfer positions
    public static final double kHomePos = 0.1;
    public static final double kWaitForCoralPos = 3;
    public static final double kInterPos = 5;

    // Scoring positions
    public static final double kProcessorPos = -5.977;
    public static final double kL1Pos = -5.870;
    public static final double kL2Pos = -4.83;
    public static final double kL3Pos = 4.4;
    public static final double kL4Pos = 19.45;
    public static final double kBargePos = 25.52;

    // Algae pickup positions
    public static final double kAlgaeFloorPos = -5.977;
    public static final double kAlgaeL2Pos = 2.66;
    public static final double kAlgaeL3Pos = 13.8;
  }

  public static final class ArmConstants {
    private ArmConstants() {}

    public static final int kArmMotorID = 16;
    public static final int kCanCoderID = 17;

    // Motion Magic gains
    public static final double kP = 6.3;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;
    public static final double kCruiseVelocity = 270;
    public static final double kAcceleration = 120;
    public static final double kStatorCurrentLimit = 120;
    public static final double kSupplyCurrentLimit = 70;
    public static final double kSensorToMechanismRatio = 3;
    public static final double kRotorToSensorRatio = 25;

    public static final double kHomePos = -0.0012;

    // Scoring positions
    public static final double kProcessorPos = 0.2357;
    public static final double kL1Pos = 0.303;
    public static final double kL2Pos = 0.358;
    public static final double kL3Pos = 0.376;
    public static final double kL4Pos = 0.4;
    public static final double kBargePos = 0.445;

    /** How far the arm drops from the scoring position to release coral. */
    public static final double kL1ReleaseOffset = 0.045;
    public static final double kCoralReleaseOffset = 0.07;

    /** Arm position used to release coral on L4 during autonomous. */
    public static final double kAutoL4ReleasePos = 0.33;

    /** Position the arm moves to once an algae is secured in the gripper. */
    public static final double kAlgaeHoldPos = 0.27;

    // Algae pickup positions
    public static final double kAlgaeFloorPos = 0.1857;
    public static final double kAlgaeL2Pos = 0.255;
    public static final double kAlgaeL3Pos = 0.255;
  }

  public static final class GripperConstants {
    private GripperConstants() {}

    public static final int kGripperMotorID = 8;
    public static final int kBreakBeamID = 0;

    public static final int kSmartCurrentLimit = 40;

    public static final double kCoralPassivePow = 0;
    public static final double kAlgaePassivePow = 0.15;
    public static final double kIntakePow = 1;
    public static final double kOuttakePow = -1;
    public static final double kStopPow = 0;
  }

  public static final class IntakeConstants {
    private IntakeConstants() {}

    public static final int kIntakeMotorID = 4;
    public static final int kPivotMotorID = 3;

    // Pivot PID gains
    public static final double kP = 0.9;
    public static final double kI = 0;
    public static final double kD = 0.4;
    public static final int kSmartCurrentLimitPivot = 40;
    public static final int kSmartCurrentLimitIntake = 40;

    // Pivot positions
    public static final double kIntakePos = 0;
    public static final double kParallelPos = 4;
    public static final double kStowedPos = 13.25;

    public static final double kIntakePow = 0.75;
    public static final double kOuttakePow = -1;
    public static final double kStopPow = 0;
  }

  public static final class IndexerConstants {
    private IndexerConstants() {}

    public static final int kLeftIndexerMotorID = 6;
    public static final int kRightIndexerMotorID = 5;
    public static final int kBreakBeamID = 2;

    public static final int kSmartCurrentLimit = 40;

    public static final double kIntakePow = 0.5;
    public static final double kOuttakePow = -0.5;
    public static final double kStopPow = 0;
  }

  public static final class ClimberConstants {
    private ClimberConstants() {}

    public static final int kClimberMotorID = 2;
    public static final int kCageMotorID = 7;

    // Climber PID gains
    public static final double kP = 0.028;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int kSmartCurrentLimit = 100;

    // Climber positions
    public static final double kHomePos = 0;
    public static final double kDeployPos = 172;
    public static final double kClimbPos = 411;

    public static final double kManualClimbPow = 1;
    public static final double kCageIntakePow = -1;
    public static final double kCageStopPow = 0;
  }

  /**
   * Reef alignment using the Limelight's robot pose in AprilTag target space. The X controller
   * drives robot-relative forward speed from the target-space Z component, the Y controller drives
   * strafe speed from the target-space X component, and the rotation controller uses the yaw.
   */
  public static final class AutoAlignConstants {
    private AutoAlignConstants() {}

    public static final String kLimelightName = "";

    public static final double kXP = 1;
    public static final double kYP = 0.18;
    public static final double kRotP = 0.1;

    /** Extra scaling applied to the rotation controller output. */
    public static final double kRotOutputScale = 0.2;

    public static final double kRightXSetpoint = 0.11;
    public static final double kRightYSetpoint = -0.56;
    public static final double kRightRotSetpoint = 3.24;

    public static final double kLeftXSetpoint = -0.25;
    public static final double kLeftYSetpoint = 0.53;
    public static final double kLeftRotSetpoint = 1.9;

    public static final double kXTolerance = 0.02;
    public static final double kYTolerance = 0.02;
    public static final double kRotTolerance = 1;

    /** Give up if the target tag is not seen for this long (s). */
    public static final double kLostTagTimeout = 3;

    /** The robot must hold the aligned pose for this long (s) before the command ends. */
    public static final double kPoseValidationTime = 0.3;
  }
}
