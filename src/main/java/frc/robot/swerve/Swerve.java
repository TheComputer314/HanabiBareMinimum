package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Swerve {
  private final Module[] modules;

  private final SwerveDriveKinematics kinematics;

  // ホイールからホイールの幅、メートル
  public static final double trackWidthMeters = .5;
  public static final double trackLengthMeters = .5;

  // ドライブベースの最大速度。秒速メートルと秒速ラジアン
  public static final double maxLinearVelocityMetersPerSec = Module.wheelMaxLinearVelocity;
  public static final double maxAngularVelocityRadiansPerSec =
      Module.wheelMaxLinearVelocity / Math.hypot(trackLengthMeters / 2, trackWidthMeters / 2);

  public Swerve() {
    modules = new Module[] {new Module(0), new Module(1), new Module(2), new Module(3)};

    // +X=前,+Y=左
    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(trackWidthMeters / 2, trackLengthMeters / 2),
            new Translation2d(trackWidthMeters / 2, -trackLengthMeters / 2),
            new Translation2d(-trackWidthMeters / 2, trackLengthMeters / 2),
            new Translation2d(-trackWidthMeters / 2, -trackLengthMeters / 2));
  }

  public void drive(ChassisSpeeds desiredSpeeds) {
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
