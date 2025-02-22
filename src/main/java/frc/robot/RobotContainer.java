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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.PositionSubsystem;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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


    public RobotContainer() {

        var elevatorMotorLeader = new SparkMax(1, MotorType.kBrushless);
        elevatorMotorLeader.configure(
            new SparkMaxConfig()
                .apply(new ClosedLoopConfig()
                    .p(0.4)
                    .i(0)
                    .d(0)
                    .apply(new MAXMotionConfig()
                        .maxVelocity(6000)
                        .maxAcceleration(12000)
                        .allowedClosedLoopError(1))),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        var elevatorMotorFollower = new SparkMax(2, MotorType.kBrushless);
        elevatorMotorFollower.configure(
            new SparkMaxConfig()
                .follow(elevatorMotorLeader, true),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        elevator = new PositionSubsystem(elevatorMotorLeader.getClosedLoopController(),
            () -> elevatorMotorLeader.getEncoder().getVelocity());

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Dynareef", Commands.deferredProxy(Dynareef::buildAuto));
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.a().whileTrue(drivetrain.applyRequest(() -> {
            var tx = LimelightHelpers.getTX("limelight-reef");
            var ta = LimelightHelpers.getTA("limelight-reef");
//            var offset = ta * 1.51 + 5.57;
            var offset = 20;
            var sideways = (tx - offset * Math.signum(tx)) * -0.2;
            sideways = MathUtil.clamp(sideways, -0.5, 0.5);
            var rotationError = LimelightHelpers.getCameraPose_TargetSpace("limelight-reef")[4];
            var rotation = rotationError * 0.1;
            //rotation = MathUtil.clamp(rotation, -0.3, 0.3);
            return new SwerveRequest.RobotCentric()
                .withRotationalDeadband(0.5)
                .withDeadband(0.5)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(sideways)
                .withRotationalRate(rotation);
        }));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        // joystick.x().whileTrue(Commands.parallel(
        //     elevator.goToPosition(50),
        //     wrist.goToPosition(40)));
        // joystick.y().whileTrue(Commands.parallel(
        //     elevator.goToPosition(100),
        //     wrist.goToPosition(80)));
        joystick.x().whileTrue(elevator.goToPosition(360));
        // joystick.y().whileTrue(pickup.run(0.2));
//        joystick.rightTrigger().whileTrue(pickup.run(0.2));
//        joystick.leftTrigger().whileTrue(pickup.run(-0.2));
        joystick.povUp().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(0.45)));
        joystick.povDown().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-0.45)));
        joystick.povRight().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityY(-0.45)));
        joystick.povLeft().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityY(0.45)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
