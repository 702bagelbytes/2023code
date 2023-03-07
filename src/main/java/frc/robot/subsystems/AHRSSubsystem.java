package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AHRSSubsystem extends SubsystemBase {
    private final AHRS ahrs = new AHRS();

    public AHRSSubsystem() {
    }

    public Rotation2d getRotation2d() {

        return ahrs.getRotation2d();
    }

    public double getRoll() {
        return ahrs.getRoll();
    }

    public double getBalanceAngle() {
        // "taking off" should translate to a positive angle being returned from the
        
        // we're stuck with a faulty encoder: add +2 to get
        // gyro reads closer to IRL.
        final double BAKED_IN_OFFSET = 0;// 1.1, 2;

        return getRoll() + BAKED_IN_OFFSET;
    }

    @Override
    public void periodic() {
        String msg = String.format("Y: %.3f, P: %.3f, R: %.3f (raw=%.3f)", ahrs.getYaw(), ahrs.getPitch(), getBalanceAngle(), ahrs.getRoll());
        SmartDashboard.putString("Gyro", msg);

        // SmartDashboard.putNumber("Yaw", ahrs.getYaw());
        // SmartDashboard.putNumber("Pitch", ahrs.getPitch());
        // SmartDashboard.putNumber("Roll", ahrs.getRoll());
    }
}
