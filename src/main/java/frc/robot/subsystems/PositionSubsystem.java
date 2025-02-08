package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final TrapezoidProfile profile;

    private final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public PositionSubsystem(TalonFX motor, TrapezoidProfile profile){
        this.motor = motor;
        this.profile = profile;
        motor.getConfigurator().apply(
            new Slot0Configs().withKP(1)
        );
    }

    @Override
    public void periodic() {
        final double TIME_STEP = 0.02;
        setpoint = profile.calculate(TIME_STEP, setpoint, goal);
        request.Position = setpoint.position;
        request.Velocity = setpoint.velocity;
        motor.setControl(request);
    }

    public Command goToPosition(double position) {
        return startEnd(
            () -> goal = new TrapezoidProfile.State(position, 0),
            () -> goal = new TrapezoidProfile.State(0, 0));
    }
}
