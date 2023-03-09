package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;

public class AHRSSubsystem extends SubsystemBase {
    private final AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

    public AHRSSubsystem() {
    }

    public Rotation2d getRotation2d() {

        return ahrs.getRotation2d();
    }

    public float getRoll() {
        return ahrs.getRoll();
    }

    public float getBalanceAngle() {
        // "taking off" should translate to a positive angle being returned from the
        return -ahrs.getYaw();
    }

    @Override
    public void periodic() {
        String msg = String.format("Y: %.3f, P: %.3f, R: %.3f", ahrs.getYaw(), ahrs.getPitch(), ahrs.getRoll());
        SmartDashboard.putString("Gyro", msg);
        SmartDashboard.putBoolean("Is gyro Connected", ahrs.isConnected());
        SmartDashboard.putNumber("Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("Pitch", ahrs.getPitch());
        SmartDashboard.putNumber("Roll", ahrs.getRoll());
    }
}
