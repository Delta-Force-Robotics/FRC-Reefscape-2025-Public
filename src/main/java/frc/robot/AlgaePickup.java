package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

/** Algae pickup locations, ordered from lowest to highest, with the matching setpoints. */
public enum AlgaePickup {
  FLOOR(ElevatorConstants.kAlgaeFloorPos, ArmConstants.kAlgaeFloorPos),
  L2(ElevatorConstants.kAlgaeL2Pos, ArmConstants.kAlgaeL2Pos),
  L3(ElevatorConstants.kAlgaeL3Pos, ArmConstants.kAlgaeL3Pos);

  private static final AlgaePickup[] kValues = values();

  public final double elevatorPos;
  public final double armPos;

  AlgaePickup(double elevatorPos, double armPos) {
    this.elevatorPos = elevatorPos;
    this.armPos = armPos;
  }

  /** The next higher pickup, or this one if already at the top. */
  public AlgaePickup next() {
    return kValues[Math.min(ordinal() + 1, kValues.length - 1)];
  }

  /** The next lower pickup, or this one if already at the bottom. */
  public AlgaePickup previous() {
    return kValues[Math.max(ordinal() - 1, 0)];
  }
}
