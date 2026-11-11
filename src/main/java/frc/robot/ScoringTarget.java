package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

/** Scoring locations, ordered from lowest to highest, with the matching elevator and arm setpoints. */
public enum ScoringTarget {
  PROCESSOR(ElevatorConstants.kProcessorPos, ArmConstants.kProcessorPos, 0),
  L1(ElevatorConstants.kL1Pos, ArmConstants.kL1Pos, ArmConstants.kL1ReleaseOffset),
  L2(ElevatorConstants.kL2Pos, ArmConstants.kL2Pos, ArmConstants.kCoralReleaseOffset),
  L3(ElevatorConstants.kL3Pos, ArmConstants.kL3Pos, ArmConstants.kCoralReleaseOffset),
  L4(ElevatorConstants.kL4Pos, ArmConstants.kL4Pos, ArmConstants.kCoralReleaseOffset),
  BARGE(ElevatorConstants.kBargePos, ArmConstants.kBargePos, 0);

  private static final ScoringTarget[] kValues = values();

  public final double elevatorPos;
  public final double armPos;
  private final double armReleaseOffset;

  ScoringTarget(double elevatorPos, double armPos, double armReleaseOffset) {
    this.elevatorPos = elevatorPos;
    this.armPos = armPos;
    this.armReleaseOffset = armReleaseOffset;
  }

  /** Arm position used when releasing the game piece at this target. */
  public double armReleasePos() {
    return armPos - armReleaseOffset;
  }

  /** The next higher target, or this one if already at the top. */
  public ScoringTarget next() {
    return kValues[Math.min(ordinal() + 1, kValues.length - 1)];
  }

  /** The next lower target, or this one if already at the bottom. */
  public ScoringTarget previous() {
    return kValues[Math.max(ordinal() - 1, 0)];
  }
}
