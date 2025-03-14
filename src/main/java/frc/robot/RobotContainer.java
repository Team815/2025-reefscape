// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private final Led led = new Led(9, 47);

    private final PositionSubsystem elevator;

    private final PositionSubsystem wrist;

    public final ClawThingy dispenser = new ClawThingy(
        new SparkMax(3, MotorType.kBrushless),
        new DigitalInput(0),
        new DigitalInput(1));

    private final MotorSubsystem climber = new MotorSubsystem(new SparkMax(5, MotorType.kBrushless));

    private boolean autoRan = false;

    public RobotContainer() {
        var elevatorMotorLeader = new SparkMax(1, MotorType.kBrushless);
        elevatorMotorLeader.configure(
            new SparkMaxConfig()
                .apply(new ClosedLoopConfig()
                    .p(0.4)
                    .i(0)
                    .d(0)
                    .apply(new MAXMotionConfig()
                        .maxVelocity(3000)
                        .maxAcceleration(6000)
                        .allowedClosedLoopError(0.2))),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        var elevatorMotorFollower = new SparkMax(2, MotorType.kBrushless);
        elevatorMotorFollower.configure(
            new SparkMaxConfig()
                .follow(elevatorMotorLeader, true),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        elevator = new PositionSubsystem(elevatorMotorLeader, 0.3, false);

        var wristMotor = new SparkMax(4, MotorType.kBrushless);
        wristMotor.configure(
            new SparkMaxConfig()
                .apply(new ClosedLoopConfig()
                    .p(0.4)
                    .i(0)
                    .d(0)
                    .apply(new MAXMotionConfig()
                        .maxVelocity(2000)
                        .maxAcceleration(4000)
                        .allowedClosedLoopError(0.2))),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        wrist = new PositionSubsystem(wristMotor, 0, false);

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Dynareef", Commands.deferredProxy(() -> Dynareef.buildAuto(this)));
        SmartDashboard.putData("Auto Mode", autoChooser);

        new EventTrigger("StartElevator").onTrue(Commands.print("Raise elevator"));
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                    var maxSpeed = getHeightDependentSpeed(MaxSpeed, 6, 26, drive::withDeadband);
                    var maxAngularRate = getHeightDependentSpeed(MaxAngularRate, 2, 26, drive::withRotationalDeadband);
                    return drive.withVelocityX(-joystick.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-joystick.getLeftX() * maxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * maxAngularRate); // Drive counterclockwise with negative X (left)
                }
            )
        );
        joystick.a().and(joystick.leftTrigger().negate()).onTrue(goToCoral1());
        joystick.a().and(joystick.leftTrigger().negate()).onFalse(goToHomePosition());
        joystick.x().and(joystick.leftTrigger().negate()).onTrue(goToCoral2());
        joystick.x().and(joystick.leftTrigger().negate()).onFalse(goToHomePosition());
        joystick.y().and(joystick.leftTrigger().negate()).onTrue(goToCoral3());
        joystick.y().and(joystick.leftTrigger().negate()).onFalse(goToHomePosition());
        joystick.x().and(joystick.leftTrigger()).onTrue(goToAlgaeLow());
        joystick.x().and(joystick.leftTrigger()).onFalse(goToProcessor());
        joystick.y().and(joystick.leftTrigger()).onTrue(goToAlgaeHigh());
        joystick.y().and(joystick.leftTrigger()).onFalse(goToProcessor());
        joystick.b().and(joystick.leftTrigger().negate()).onTrue(finalizeStationPosition());
        joystick.b().and(joystick.leftTrigger()).whileTrue(climber.run(-0.7));
        joystick.a().and(joystick.leftTrigger()).whileTrue(climber.run(0.7));
        joystick.rightTrigger().whileTrue(runDispenser(-1));
        joystick.rightBumper().whileTrue(runDispenser(1));
        joystick.leftBumper().whileTrue(finalizeReefPosition());

        joystick.povUp().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(0.45)));
        joystick.povDown().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-0.45)));
        joystick.povRight().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityY(-0.45)));
        joystick.povLeft().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityY(0.45)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
//        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
//        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
//        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
//        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.runOnce(() -> autoRan = true).andThen(autoChooser.getSelected());
//        return goToLevel4()
//            .andThen(dispenser.run(-1).withTimeout(1))
//            .andThen(goToHomePosition());
    }

    public Command fixFieldCentric() {
        return Commands.either(
            Commands.runOnce(() -> drivetrain.resetRotation(drivetrain.getState().Pose.getRotation().plus(Rotation2d.k180deg))),
            Commands.none(),
            () -> autoRan && DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
        ).andThen(Commands.runOnce(() -> autoRan = false));
    }

    public Command goToCoral0() {
        return goToPosition(0, -7, true);
    }

    public Command goToCoral1() {
        return goToPosition(18, -7, true);
    }

    public Command goToCoral2() {
        return goToPosition(54, -7, true);
    }

    public Command goToCoral3() {
        return goToPosition(119, -12, true);
    }

    public Command goToAlgaeHigh() {
        return goToPosition(90, -42, false);
    }

    public Command goToAlgaeLow() {
        return goToPosition(58, -42, false);
    }

    public Command goToProcessor() {
        return goToPosition(40, -50, false);
    }

    public Command goToHomePosition() {
        return goToPosition(0, -2, false);
    }

    private Command goToPosition(double elevatorTarget, double wristTarget, boolean holdingCoral) {
//        return Commands.print("First").andThen(Commands.print("Second"), Commands.print("Third"));
        var command = Commands.either(

            // Elevator rising

            Commands.either(

                // Elevator not rising very high

                elevator.goToPosition(elevatorTarget)
                    .alongWith(
                        wrist.goToPosition(wristTarget),
                        Commands.print("Elevator rising a little")),

                // Elevator rising high

                elevator.goToPosition(26)
                    .alongWith(
                        wrist.goToPosition(wristTarget),
                        Commands.print("Elevator rising high"))
                    .withDeadline(Commands.waitUntil(() -> wristTarget < -6))
                    .andThen(elevator.goToPosition(elevatorTarget))
                    .withDeadline(Commands.waitUntil(() -> elevator.isAtPosition() && wrist.isAtPosition())),
                () -> elevatorTarget <= 26),

            // Elevator falling

            Commands.either(

                // Wrist extending

                wrist.goToPosition(wristTarget)
                    .alongWith(
                        elevator.goToPosition(Math.max(26, elevatorTarget)),
                        Commands.print("Elevator falling"),
                        Commands.print("Wrist extending"),
                        Commands.waitUntil(() -> wrist.getPosition() < -6 || wrist.isAtPosition()))
                    .andThen(elevator.goToPosition(elevatorTarget)),

                // Wrist retracting

                wrist.goToPosition(-7)
                    .alongWith(
                        Commands.print("Elevator falling"),
                        Commands.print("Wrist retracting"),
                        Commands.waitUntil(() -> wrist.getPosition() > -30)
                            .andThen(elevator.goToPosition(elevatorTarget)),
                        Commands.waitUntil(() -> elevator.getPosition() < 26)
                    ).andThen(wrist.goToPosition(wristTarget)
                        .withDeadline(Commands.waitUntil(() -> elevator.isAtPosition() && wrist.isAtPosition()))),
                () -> wrist.getPosition() > wristTarget
            ),
            () -> elevator.getPosition() < elevatorTarget
        );

        return Commands.either(
            dispenser.receiveCoral(),
            Commands.none(),
            () -> holdingCoral
        ).andThen(command);
    }

    private Command runDispenser(double speed) {
        return Commands.either(
            dispenser.run(speed),
            dispenser.run(speed)
                .withDeadline(Commands.waitUntil(dispenser::hasCoral)),
            dispenser::hasCoral
        );
    }

    public Command finalizeReefPosition() {
        final double taTarget = 18;
        return drivetrain.applyRequest(() -> {
                var limelightName = LimelightHelpers.getTA("limelight-reefl") > LimelightHelpers.getTA("limelight-reefr")
                    ? "limelight-reefl"
                    : "limelight-reefr";
                var maxSpeed = getHeightDependentSpeed(MaxSpeed, 6, 26, __ -> {
                });
                var tx = LimelightHelpers.getTX(limelightName);
                var ta = LimelightHelpers.getTA(limelightName);
                var taError = taTarget - ta;
                var cameraPose = LimelightHelpers.getCameraPose_TargetSpace(limelightName);
                var rotationError = cameraPose.length > 4 ? cameraPose[4] : 0;
                var rotation = ta < 20 ? 0 : rotationError * 0.1;
                var speedX = ta == 0 ? 0 : taError * 0.036;
                return new SwerveRequest
                    .RobotCentric()
                    .withVelocityY(MathUtil.clamp(-tx * 0.05, -maxSpeed, maxSpeed))
                    .withVelocityX(MathUtil.clamp(speedX, -maxSpeed, maxSpeed))
                    .withRotationalRate(rotation)
                    .withDeadband(0.1)
                    .withRotationalDeadband(0.1);
            }
        ).withDeadline(Commands.waitUntil(() -> {
            var limelightName = LimelightHelpers.getTA("limelight-reefl") > LimelightHelpers.getTA("limelight-reefr")
                ? "limelight-reefl"
                : "limelight-reefr";
            var tx = LimelightHelpers.getTX(limelightName);
            var ta = LimelightHelpers.getTA(limelightName);
            var taError = taTarget - ta;
            return tx != 0
                && ta != 0
                && Math.abs(tx) < 2
                && Math.abs(taError) < 4;
        }));
    }

    public Command finalizeStationPosition() {
        return drivetrain.applyRequest(() -> {
                var limelightName = "limelight-station";
                var maxSpeed = getHeightDependentSpeed(MaxSpeed, 6, 26, __ -> {
                });
                var tx = LimelightHelpers.getTX(limelightName);
                var ty = LimelightHelpers.getTY(limelightName);
                return new SwerveRequest
                    .RobotCentric()
                    .withVelocityY(MathUtil.clamp(tx * 0.05, -maxSpeed, maxSpeed))
                    .withVelocityX(MathUtil.clamp(ty * 0.05, -maxSpeed, maxSpeed))
                    .withDeadband(0.1);
            }
        ).withDeadline(Commands.waitUntil(() -> {
            var limelightName = "limelight-station";
            var tx = LimelightHelpers.getTX(limelightName);
            var ty = LimelightHelpers.getTY(limelightName);
            return tx != 0
                && ty != 0
                && Math.abs(tx) < 2
                && Math.abs(ty) < 2;
        }));
    }

    private double getHeightDependentSpeed(
        double maxSpeed,
        double reduction,
        double heightThreshold,
        Consumer<Double> deadbandSetter) {
        if (elevator.getPosition() > heightThreshold) {
            maxSpeed /= reduction;
        } else {
            maxSpeed *= 0.9;
        }
        deadbandSetter.accept(maxSpeed * 0.1);
        return maxSpeed;
    }
}
