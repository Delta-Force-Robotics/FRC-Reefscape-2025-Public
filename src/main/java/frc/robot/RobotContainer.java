// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.DriveAfterScoreCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final GripperSubsystem gripperSubsystem = new GripperSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private DriveAfterScoreCommand driveAfterScoreCommand = new DriveAfterScoreCommand(drivetrain, 1, 0.55);
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors     

    public final CommandPS5Controller driver = new CommandPS5Controller(0);
    private final CommandPS5Controller operator = new CommandPS5Controller(1);
    private final CommandPS5Controller driver3 = new CommandPS5Controller(2);

    private final Trigger indexerBreakBeamTripped = new Trigger(indexerSubsystem.isBreakBeamTripped());
    private final Trigger gripperBreakBeamTripped = new Trigger(gripperSubsystem.isBreakBeamTripped());

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        new EventTrigger("ExtendScoringL4").onTrue(
            Commands.sequence(
                elevatorSubsystem.moveElevator(ElevatorConstants.kL4Pos),
                new WaitCommand(0.15),
                armSubsystem.moveArm(ArmConstants.kL4Pos)
            )
        );

        new EventTrigger("ScoreCoralL4").onTrue(
            Commands.sequence(
                armSubsystem.moveArm(0.33),
                new WaitCommand(0.15),
                new InstantCommand(() -> gripperSubsystem.intakingAlgae = false),
                gripperSubsystem.runGripper(GripperConstants.kOuttakePow)                
            )
        );

        new EventTrigger("RetractScore").onTrue(
            Commands.sequence(
            armSubsystem.moveArm(ArmConstants.kHomePos),
            new WaitCommand(0.2),
            elevatorSubsystem.moveElevator(ElevatorConstants.kWaitForCoralPos),
            gripperSubsystem.runGripper(GripperConstants.kStopPow)
            )
        );

        new EventTrigger("StartIntake").onTrue(
            Commands.sequence(
                intakeSubsystem.moveIntake(IntakeConstants.kIntakePos),
                intakeSubsystem.runIntake(IntakeConstants.kIntakePow),
                indexerSubsystem.runIndexer(IndexerConstants.kIntakePow)
            )
        );

        new EventTrigger("Reverse&StopIntake").onTrue(
            Commands.sequence(
                intakeSubsystem.moveIntake(IntakeConstants.kParallelPos),
                intakeSubsystem.runIntake(IntakeConstants.kOuttakePow),
                indexerSubsystem.runIndexer(IndexerConstants.kOuttakePow),
                new WaitCommand(0.2),
                intakeSubsystem.moveIntake(IntakeConstants.kStowedPos),
                intakeSubsystem.runIntake(IntakeConstants.kStopPow),
                indexerSubsystem.runIndexer(IndexerConstants.kStopPow)
            )
        );

        new EventTrigger("IntakeAlgae2").onTrue(
            Commands.sequence(
                elevatorSubsystem.moveElevator(ElevatorConstants.kAlgaeL2Pos),
                new WaitCommand(0.15),
                armSubsystem.moveArm(ArmConstants.kAlgaeL2Pos))
        );

        new EventTrigger("IntakeAlgae3").onTrue(
            Commands.sequence(
                gripperSubsystem.runGripper(1),
                elevatorSubsystem.moveElevator(ElevatorConstants.kAlgaeL3Pos),
                new WaitCommand(0.15),
                armSubsystem.moveArm(ArmConstants.kAlgaeL3Pos))
        );

        new EventTrigger("ScoreAlgae").onTrue(
            Commands.sequence(
                elevatorSubsystem.moveElevator(ElevatorConstants.kBargePos),
                new WaitCommand(0.15),
                armSubsystem.moveArm(ArmConstants.kBargePos))
        );

        new EventTrigger("ReleaseAlgae").onTrue(
            gripperSubsystem.runGripper(-1)
        );

        autoChooser = AutoBuilder.buildAutoChooser("Test");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * elevatorSubsystem.multiplier())
                .withVelocityY(-driver.getLeftX() * MaxSpeed * elevatorSubsystem.multiplier())
                .withRotationalRate(-driver.getRightX() * MaxAngularRate * elevatorSubsystem.multiplier())
            )
        );

        // Reset field centric
        driver.options().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        // Start coral pick-up when tripped
        indexerBreakBeamTripped.onTrue(
            Commands.sequence(
            new WaitCommand(0.1),
            gripperSubsystem.runGripper(GripperConstants.kIntakePow),
            armSubsystem.moveArm(ArmConstants.kHomePos),
            elevatorSubsystem.moveElevator(ElevatorConstants.kHomePos),
            new WaitCommand(0.15),
            gripperSubsystem.runPassiveGripper(GripperConstants.kCoralPassivePow),
            elevatorSubsystem.moveElevator(ElevatorConstants.kInterPos),
            intakeSubsystem.moveIntake(IntakeConstants.kParallelPos),
            intakeSubsystem.runIntake(IntakeConstants.kOuttakePow),
            indexerSubsystem.runIndexer(IndexerConstants.kOuttakePow),
            new WaitCommand(0.1),
            intakeSubsystem.moveIntake(IntakeConstants.kStowedPos),
            intakeSubsystem.runIntake(IntakeConstants.kStopPow),
            indexerSubsystem.runIndexer(IndexerConstants.kStopPow)
            )
        );

         // Start gripper passive power when tripped
        gripperBreakBeamTripped.onTrue(
            Commands.sequence(
                gripperSubsystem.runPassiveGripper(GripperConstants.kAlgaePassivePow),
                new InstantCommand(() -> elevatorSubsystem.setMultiplier(1)),
                armSubsystem.moveArm(0.27)
            ));

        // Start intake
        driver.cross().onTrue(
            Commands.sequence(
                new InstantCommand(() -> gripperSubsystem.intakingAlgae = false),
                elevatorSubsystem.moveElevator(ElevatorConstants.kInterPos),
                intakeSubsystem.moveIntake(IntakeConstants.kParallelPos),
                new WaitCommand(0.3),
                intakeSubsystem.moveIntake(IntakeConstants.kIntakePos),
                intakeSubsystem.runIntake(IntakeConstants.kIntakePow),
                indexerSubsystem.runIndexer(IndexerConstants.kIntakePow),
                armSubsystem.moveArm(ArmConstants.kHomePos),
                elevatorSubsystem.moveElevator(ElevatorConstants.kWaitForCoralPos)
            )
        );

        // Stop intake
        driver.circle().onTrue(
            Commands.sequence(
                intakeSubsystem.moveIntake(IntakeConstants.kStowedPos),
                intakeSubsystem.runIntake(IntakeConstants.kOuttakePow),
                indexerSubsystem.runIndexer(IndexerConstants.kOuttakePow),
                new WaitCommand(0.35),
                intakeSubsystem.runIntake(IntakeConstants.kStopPow),
                indexerSubsystem.runIndexer(IndexerConstants.kStopPow)
            )
        );

        // Intake algae
        driver.square().onTrue(
            Commands.sequence(
                new InstantCommand(() -> gripperSubsystem.intakingAlgae = true),
                elevatorSubsystem.moveElevator(ElevatorConstants.kInterPos),
                new WaitCommand(0.25),
                armSubsystem.pickUpAlgae(),
                new WaitCommand(0.15),
                elevatorSubsystem.pickUpAlgae(),
                gripperSubsystem.runGripper(GripperConstants.kIntakePow)
            )
        );

        // Extend scoring
        driver.L1().onTrue(
            Commands.sequence(
                new InstantCommand(() -> elevatorSubsystem.setMultiplier(0.3)),
                elevatorSubsystem.moveElevator(ElevatorConstants.kInterPos),
                new WaitCommand(0.25),
                armSubsystem.goToTarget(),
                new WaitCommand(0.15),
                elevatorSubsystem.goToTarget()
            )
        );

        // Score element
        driver.R1().onTrue(
            Commands.sequence(
                new InstantCommand(() -> elevatorSubsystem.setMultiplier(1)),
                new InstantCommand(() -> armSubsystem.scoreRelease()),
                new WaitCommand(0.15),
                new InstantCommand(() -> gripperSubsystem.intakingAlgae = false),
                gripperSubsystem.runGripper(GripperConstants.kOuttakePow),
                new WaitCommand(0.2),
                driveAfterScoreCommand,
                new WaitCommand(0.25),
                elevatorSubsystem.moveElevator(ElevatorConstants.kInterPos),
                new WaitCommand(0.1),
                armSubsystem.moveArm(ArmConstants.kHomePos),
                new WaitCommand(0.25),
                elevatorSubsystem.moveElevator(ElevatorConstants.kWaitForCoralPos),
                gripperSubsystem.runGripper(GripperConstants.kStopPow)
            )
        );

        driver.povLeft().onTrue(
            Commands.sequence(
                climbSubsystem.moveClimber(ClimberConstants.kDeployPos),
                climbSubsystem.runCageMotor(-1)
            )
         );  

         driver.povRight().onTrue(
            Commands.sequence(
                climbSubsystem.moveClimber(ClimberConstants.kClimbPos),
                climbSubsystem.runCageMotor(0)
            )        );  

        // Operator emergency retract intake
        operator.circle().onTrue(
            Commands.sequence(
                intakeSubsystem.runIntake(IntakeConstants.kStopPow),
                indexerSubsystem.runIndexer(IndexerConstants.kStopPow),
                intakeSubsystem.moveIntake(IntakeConstants.kStowedPos)
            )
        );

        // Increment scoring and pick-up positions
        operator.povUp().onTrue(
            new InstantCommand(
                () -> {
                    elevatorSubsystem.increaseScoringIncrement();
                    armSubsystem.increaseScoringTarget();
                }
            )
        );

        operator.povDown().onTrue(
            new InstantCommand(
                () -> {
                    elevatorSubsystem.decreaseScoringIncrement();
                    armSubsystem.decreaseScoringTarget();
                }
            )
        );

        operator.triangle().onTrue(
            new InstantCommand(
                () -> {
                    elevatorSubsystem.increaseAlgaeIncrement();
                    armSubsystem.increaseAlgaePickUp();
                }
            )
        );

        operator.cross().onTrue(
            new InstantCommand(
                () -> {
                    elevatorSubsystem.decreaseAlgaeIncrement();
                    armSubsystem.decreaseAlgaePickUp();
                }
            )
        );

        driver3.cross().onTrue(
            climbSubsystem.manualClimb(1)
        );

        driver3.circle().onTrue(
            climbSubsystem.manualClimb(0)
        );

        driver3.square().onTrue(
            climbSubsystem.manualClimb(-1)
        );

    //  operator.square().onTrue(
    //          intakeSubsystem.resetEncoder()
    //      );  
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}