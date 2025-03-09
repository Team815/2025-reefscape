package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionSubsystem extends SubsystemBase {
    private final SparkClosedLoopController controller;
    private final RelativeEncoder encoder;
    private final boolean print;
    private double setpoint;
    private double feedForward;

    public PositionSubsystem(SparkMax motor, double feedForward, boolean print) {
        controller = motor.getClosedLoopController();
        encoder = motor.getEncoder();
        this.feedForward = feedForward;
        this.print = print;
    }

    @Override
    public void periodic() {
        if (!print) {
            return;
        }
        System.out.printf("Position: %.2f, Velocity: %.2f\n", encoder.getPosition(), encoder.getVelocity());
    }

    public Command goToPosition(double position) {
        return runOnce(() -> setPosition(position));
    }

    public boolean isAtPosition() {
        return Math.abs(encoder.getPosition() - setpoint) < 1;
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    private void setPosition(double position){
        setpoint = position;
        controller.setReference(
            position,
            SparkBase.ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0,
            feedForward);
    }
}
