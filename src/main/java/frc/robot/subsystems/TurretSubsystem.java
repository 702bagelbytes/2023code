package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    private final WPI_TalonFX talon = new WPI_TalonFX(Constants.TelescopeConstants.kExtensionTalonFX);
    // // private final Encoder encoder = new Encoder(0, 0);

    public TurretSubsystem() {
    }

    public void set(double value) {
        talon.set(value * 0.5);
    }

    private static double clampSpeed(double numRotations, double speed) {
        final double MAX_LEFT = -.5;
        final double MAX_RIGHT = .5;

        final double[][] SLOWING_THRESHOLD = {
                { .1, 1 / 2 },
                { .05, 1 / 6 }
        };

        if (numRotations > MAX_RIGHT || numRotations < MAX_LEFT) {
            return 0d;
        }

        double rotationsAbs = Math.abs(numRotations);
        double slowed = .5 - rotationsAbs;

        if (slowed < SLOWING_THRESHOLD[0][0]) {
            return speed * SLOWING_THRESHOLD[0][1];
        }

        if (slowed < SLOWING_THRESHOLD[1][0]) {
            return speed * SLOWING_THRESHOLD[1][1];
        }

        return speed;
    }

    private boolean allowErrorFor(double actual, double errAmt, double expected) {
        double[] bounds = { expected + errAmt, expected - errAmt };
        return actual <= bounds[0] && actual >= bounds[1];
    }

    /**
     * FIXME: does not work
     */
    public Command resetArm() {
        return Commands.runEnd(() -> {
            while (true) {
                final double numRotations = this.getRotations();

                double speed;
                if (numRotations > 0) {
                    speed = -.5;
                } else {
                    speed = .5;
                }

                if (allowErrorFor(numRotations, .1, 0)) {
                    return;
                }

                this.talon.set(speed);
            }
        }, () -> this.talon.set(0), this);
    }

    /**
     * turret sprockets have a 14:1 gear ratio.
     * 
     * @return the number of rotations
     */
    public double getRotations() {
        return talon.getSelectedSensorPosition() / 4096D / 36D;
    }

    public Command runCmd(DoubleSupplier input) {
        SlewRateLimiter rateLimiter = new SlewRateLimiter(0.7, -0.7, 0.0);

        Runnable onTick = () -> {
            double speed = rateLimiter.calculate(input.getAsDouble());

            double numRotations = this.getRotations();

            SmartDashboard.putNumber("number of rotations: ", numRotations);

            this.set(clampSpeed(numRotations, speed));
        };

        Runnable end = () -> this.set(0.0);

        return this.runEnd(onTick, end);

    }
}