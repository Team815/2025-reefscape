package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.DoubleSupplier;

public class PositionSubsystem extends SubsystemBase {
    private final SparkClosedLoopController controller;
    private final RelativeEncoder encoder;

    public PositionSubsystem(SparkMax motor) {
        controller = motor.getClosedLoopController();
        encoder = motor.getEncoder();
    }

    public Command stayAtPosition(double position) {
        return startEnd(
            () -> setPosition(position),
            () -> setPosition(0));
    }

    public Command goToPosition(double position) {
        return runOnce(() -> setPosition(position))
            .alongWith(new WaitUntilCommand(() -> Math.abs(encoder.getPosition() - position) < 5));
    }

    private void setPosition(double position){
        controller.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
    }
}
