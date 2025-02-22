package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class PositionSubsystem extends SubsystemBase {
    private final SparkClosedLoopController controller;
    private final DoubleSupplier velocitySupplier;

    public PositionSubsystem(SparkClosedLoopController controller, DoubleSupplier supplier) {
        this.controller = controller;
        velocitySupplier = supplier;
    }

    @Override
    public void periodic() {
        super.periodic();
        System.out.println(velocitySupplier.getAsDouble());
    }

    public Command goToPosition(double position) {
        return startEnd(
            () -> controller.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl),
            () -> controller.setReference(0, SparkBase.ControlType.kMAXMotionPositionControl));
    }
}
