package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

class Module {
  private final SparkMax driveMotor;
  private final RelativeEncoder driveEncoder;
  private final SparkMax steerMotor;
  private final CANcoder steerEncoder;

  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController drivePID;
  private final PIDController steerPID;

  // モジュールのギア比
  // TODO: セットする
  private static final double gearRatio = 5.5;
  // ホイールの直径
  private static final double wheelDiameterMeters = Units.inchesToMeters(3);
  // NEOの最大RPM
  private static final double motorMaxRPM = 5880;
  // ホイールの最大角速度。秒速回転。
  private static final double wheelMaxAngularVelocity = motorMaxRPM / (60 * gearRatio);
  // ホイールの最大線速度。秒速メートル。
  static final double wheelMaxLinearVelocity =
      wheelMaxAngularVelocity * wheelDiameterMeters * Math.PI;

  // 最大ボルテージ / 最大速度 = ボルト / 秒速メートル。すなわち、秒速1メートルのスピードだすために何ボルト必要か

  public Module(int id) {
    // エンコーダーのオフセット。0から360じゃなくて-0.5から+0.5。
    // 真っ直ぐ前にセットしてPhoenix Tunerでオフセットを測る
    double steerEncoderOffset;

    switch (id) {
      case 0 -> { // 左前
        driveMotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
        steerMotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
        steerEncoder = new CANcoder(0); // TODO: CAN IDをセットする
        steerEncoderOffset = 0; // TODO: エンコーダーのオフセットをセットする
      }
      case 1 -> { // 右前
        driveMotor = new SparkMax(1, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
        steerMotor = new SparkMax(1, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
        steerEncoder = new CANcoder(1); // TODO: CAN IDをセットする
        steerEncoderOffset = 0; // TODO: エンコーダーのオフセットをセットする
      }
      case 2 -> { // 左後ろ
        driveMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
        steerMotor = new SparkMax(3, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
        steerEncoder = new CANcoder(2); // TODO: CAN IDをセットする
        steerEncoderOffset = 0; // TODO: エンコーダーのオフセットをセットする
      }
      case 3 -> { // 右後ろ
        driveMotor = new SparkMax(3, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
        steerMotor = new SparkMax(3, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
        steerEncoder = new CANcoder(3); // TODO: CAN IDをセットする
        steerEncoderOffset = 0; // TODO: エンコーダーのオフセットをセットする
      }
      default -> throw new IndexOutOfBoundsException();
    }

    // ドライブモーターのコンフィグ
    SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    driveMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake); // 動かない時にブレーキかける
    driveMotorConfig.smartCurrentLimit(60); // モーターの電流の制御 (6０アンペアマックス)
    // モーターの回転からホイールのメートル距離への変換
    driveMotorConfig.encoder.positionConversionFactor(wheelDiameterMeters * Math.PI / gearRatio);
    // モーターのRPMからホイールの秒速メートルへの変換
    driveMotorConfig.encoder.velocityConversionFactor(
        wheelDiameterMeters * Math.PI / (gearRatio * 60));

    // ドライブモーターのセンサー
    driveEncoder = driveMotor.getEncoder();

    // コンフィグをセットする
    driveMotor.configure(
        driveMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    SparkMaxConfig steerMotorConfig = new SparkMaxConfig();
    steerMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    // NEO 550は燃えやすいから電流制御結構低くないとダメなの。
    // 30アンペアでも高いかも。
    steerMotorConfig.smartCurrentLimit(30);

    steerMotor.configure(
        steerMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    CANcoderConfiguration steerEncoderConfig = new CANcoderConfiguration();
    steerEncoderConfig.MagnetSensor.MagnetOffset = steerEncoderOffset;
    steerEncoder.getConfigurator().apply(steerEncoderConfig);

    driveFeedforward = new SimpleMotorFeedforward(0, 12 / wheelMaxLinearVelocity);
    // ホイールがターゲットの速度に行くためのフィードバック
    drivePID = new PIDController(12 / wheelMaxLinearVelocity, 0, 0);
    // ホイールがターゲットの角度に行くためのフィードバック
    steerPID = new PIDController(100, 0, .5);
    // +180度と-180度は同じだからPIDでそうセットする
    steerPID.enableContinuousInput(-.5, .5);
  }

  void run(SwerveModuleState desiredState) {
    // センサーからデータゲット
    double currentSteerAngleRotations = steerEncoder.getAbsolutePosition().getValueAsDouble();
    double currentMotorVelMetersPerSec = driveEncoder.getVelocity();

    // たまにはターゲットの速度とターゲットの角度をひっくり返すほうが速い
    // こうすればホイールの回転は最大で90度
    desiredState.optimize(Rotation2d.fromRotations(currentSteerAngleRotations));

    driveMotor.setVoltage(
        // フィードフォワードで必要なボルテージ予測し、フィードバックでターゲットからの逸脱に反応する
        driveFeedforward.calculate(desiredState.speedMetersPerSecond)
            + drivePID.calculate(
                currentMotorVelMetersPerSec, // 今の速度
                desiredState.speedMetersPerSecond)); // ターゲットの速度
    steerMotor.setVoltage(
        // フィードバックでターゲットからの逸脱に反応する
        steerPID.calculate(
            currentSteerAngleRotations, // 今のポジション
            desiredState.angle.getRotations())); // ターゲットのポジション
  }
}
