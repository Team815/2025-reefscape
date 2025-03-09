// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.PositionSubsystem;

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

    public final MotorSubsystem dispenser = new MotorSubsystem(new SparkMax(3, MotorType.kBrushless));

    private final MotorSubsystem climber = new MotorSubsystem(new SparkMax(5, MotorType.kBrushless));

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
//        joystick.b().whileTrue(drivetrain.applyRequest(() -> {
//            var tx = LimelightHelpers.getTX("limelight-reef");
//            var ta = LimelightHelpers.getTA("limelight-reef");
////            var offset = ta * 1.51 + 5.57;
//            var offset = 20;
//            var sideways = (tx - offset * Math.signum(tx)) * -0.2;
//            sideways = MathUtil.clamp(sideways, -0.5, 0.5);
//            var rotationError = LimelightHelpers.getCameraPose_TargetSpace("limelight-reef")[4];
//            var rotation = rotationError * 0.1;
//            //rotation = MathUtil.clamp(rotation, -0.3, 0.3);
//            return new SwerveRequest.RobotCentric()
//                .withRotationalDeadband(0.5)
//                .withDeadband(0.5)
//                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
//                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
//                .withVelocityY(sideways)
//                .withRotationalRate(rotation);
//        }));
        joystick.a().and(joystick.leftTrigger().negate()).onTrue(goToLevel1());
        joystick.a().and(joystick.leftTrigger().negate()).onFalse(goToHomePosition());
        joystick.x().and(joystick.leftTrigger().negate()).onTrue(goToLevel2());
        joystick.x().and(joystick.leftTrigger().negate()).onFalse(goToHomePosition());
        joystick.y().and(joystick.leftTrigger().negate()).onTrue(goToLevel3());
        joystick.y().and(joystick.leftTrigger().negate()).onFalse(goToHomePosition());
        joystick.b().and(joystick.leftTrigger().negate()).whileTrue(finalizePosition());
        joystick.x().and(joystick.leftTrigger()).whileTrue(goToPosition(48, -36));
        joystick.x().and(joystick.leftTrigger()).onFalse(goToHomePosition());
        joystick.y().and(joystick.leftTrigger()).whileTrue(goToPosition(84, -36));
        joystick.y().and(joystick.leftTrigger()).onFalse(goToHomePosition());
        joystick.rightTrigger().whileTrue(dispenser.run(-1));
        joystick.rightBumper().whileTrue(dispenser.run(1));
        joystick.b().and(joystick.leftTrigger()).whileTrue(climber.run(-3));
        joystick.a().and(joystick.leftTrigger()).whileTrue(climber.run(3));

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
        return autoChooser.getSelected();
//        return goToLevel4()
//            .andThen(dispenser.run(-1).withTimeout(1))
//            .andThen(goToHomePosition());
    }

    public Command goToLevel0() {
        return goToPosition(0, -7);
    }

    public Command goToLevel1() {
        return goToPosition(18, -7);
    }

    public Command goToLevel2() {
        return goToPosition(54, -7);
    }

    public Command goToLevel3() {
        return goToPosition(118, -8);
    }

    private Command goToPosition(double elevatorPosition, double wristPosition) {
        if (elevatorPosition <= 26) {
            return elevator.goToPosition(elevatorPosition)
                .alongWith(wrist.goToPosition(wristPosition));
        }
        return elevator.goToPosition(26)
            .alongWith(wrist.goToPosition(wristPosition))
            .withDeadline(Commands.waitUntil(() -> wristPosition < -6))
            .andThen(elevator.goToPosition(elevatorPosition))
            .withDeadline(Commands.waitUntil(() -> elevator.isAtPosition() && wrist.isAtPosition()));
    }

    public Command goToHomePosition() {
        return elevator.goToPosition(0)
            .alongWith(
                wrist.goToPosition(-7),
                Commands.waitUntil(() -> elevator.getPosition() < 26))
            .andThen(wrist.goToPosition(-1.5))
            .withDeadline(Commands.waitUntil(() -> elevator.isAtPosition() && wrist.isAtPosition()));
    }

    public Command finalizePosition() {
        return drivetrain.applyRequest(() -> {
                var limelightName = LimelightHelpers.getTA("limelight-reefl") > LimelightHelpers.getTA("limelight-reefr")
                    ? "limelight-reefl"
                    : "limelight-reefr";
                var maxSpeed = getHeightDependentSpeed(MaxSpeed, 6, 26, __ -> {
                });
                var tx = LimelightHelpers.getTX(limelightName);
                var ta = LimelightHelpers.getTA(limelightName);
                var taError = 28 - ta;
                var cameraPose = LimelightHelpers.getCameraPose_TargetSpace(limelightName);
                var rotationError = cameraPose.length > 4 ? cameraPose[4] : 0;
                var rotation = ta < 20 ? 0 : rotationError * 0.1;
                var speedX = ta == 0 ? 0 : taError * 0.036;
                System.out.println(limelightName);
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
            var taError = 28 - ta;
            return tx != 0
                && ta != 0
                && Math.abs(tx) < 0.01
                && Math.abs(taError) < 0.01;
        }));
    }

    private double getHeightDependentSpeed(
        double maxSpeed,
        double reduction,
        double heightThreshold,
        Consumer<Double> deadbandSetter) {
        if (elevator.getPosition() > heightThreshold) {
            maxSpeed /= reduction;
        }
        deadbandSetter.accept(maxSpeed * 0.1);
        return maxSpeed;
    }
}
