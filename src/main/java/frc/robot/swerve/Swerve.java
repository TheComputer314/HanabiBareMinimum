package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
  private final Module[] modules;

  private final SwerveDriveKinematics kinematics;

  // ホイールからホイールの幅、メートル
  private final double trackLengthMeters = .5;
  private final double trackWidthMeters = .5;

  // ドライブベースの最大速度。秒速メートルと秒速ラジアン
  private final double maxLinearVelocityMetersPerSec = Module.wheelMaxLinearVelocity;
  private final double maxAngularVelocityRadiansPerSec =
      Module.wheelMaxLinearVelocity / Math.hypot(trackWidthMeters / 2, trackLengthMeters / 2);

  public Swerve() {
    modules = new Module[] {new Module(0), new Module(1), new Module(2), new Module(3)};

    // +X=前,+Y=左
    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(trackLengthMeters / 2, trackWidthMeters / 2),
            new Translation2d(trackLengthMeters / 2, -trackWidthMeters / 2),
            new Translation2d(-trackLengthMeters / 2, trackWidthMeters / 2),
            new Translation2d(-trackLengthMeters / 2, -trackWidthMeters / 2));
  }

  public Command teleopDrive(
      DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier yawInput) {
    return run(
        () ->
            drive(
                // ドライバーの入力は+-1だからそれ x 最大速度 = ターゲットの速度
                new ChassisSpeeds(
                    xInput.getAsDouble() * maxLinearVelocityMetersPerSec,
                    yInput.getAsDouble() * maxLinearVelocityMetersPerSec,
                    yawInput.getAsDouble() * maxAngularVelocityRadiansPerSec)));
  }

  private void drive(ChassisSpeeds desiredSpeeds) {
    // ターゲットの速度と角度を計算
    SwerveModuleState[] targetModuleSpeeds = kinematics.toSwerveModuleStates(desiredSpeeds);
    // ターゲットの速度があまりに速すぎないように、たまにはスピードを下げる
    SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleSpeeds, Module.wheelMaxLinearVelocity);

    // ターゲットの速度と角度をセットする
    for (int i = 0; i < 4; i++) {
      modules[i].run(targetModuleSpeeds[i]);
    }
  }
}
