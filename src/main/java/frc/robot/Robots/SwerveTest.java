package frc.robot.Robots;

import java.util.Optional;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.hardware.Motor.FeedforwardConstants;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.SysIDCommands;
import frc.robot.utilities.logging.HoundLog;

public class SwerveTest extends TimedRobot {
    private Motor[] driveMotors = new Motor[] {
        Motor.fromTalonFX( // fl
            8, 
            motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(12.1908);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(new PIDController(0, 0, 0), pid -> {}), 
            Optional.empty(), 
            TargetType.Velocity
        ),
        Motor.fromTalonFX( // fr
            2, 
            motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(12.1908);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(new PIDController(0, 0, 0), pid -> {}), 
            Optional.empty(), 
            TargetType.Velocity
        ),
        Motor.fromTalonFX( // bl
            6, 
            motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(12.1908);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(new PIDController(0, 0, 0), pid -> {}), 
            Optional.empty(), 
            TargetType.Velocity
        ),
        Motor.fromTalonFX( // br
            4, 
            motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(12.1908);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(new PIDController(0, 0, 0), pid -> {}), 
            Optional.empty(), 
            TargetType.Velocity
        )
    };
    private Motor[] angleMotors = new Motor[] {
        Motor.fromSparkMax(
            9, 
            false, 
            spark -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
                config
                    .encoder
                    .positionConversionFactor(1/6.75 )
                    .velocityConversionFactor(1/6.75 / 60);
                spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            sim -> {}, 
            0, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ),
            Optional.of(new FeedforwardConstants(0, 0.58398, 0.84001, 0.14444)),
            TargetType.Position
        ),
        Motor.fromSparkMax(
            3, 
            false, 
            spark -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
                config
                    .encoder
                    .positionConversionFactor(1/6.75 )
                    .velocityConversionFactor(1/6.75 / 60);
                spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ),
            Optional.empty(),
            TargetType.Position
        ),
        Motor.fromSparkMax(
            7, 
            false, 
            spark -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
                config
                    .encoder
                    .positionConversionFactor(1/6.75 )
                    .velocityConversionFactor(1/6.75 / 60);
                spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ),
            Optional.empty(),
            TargetType.Position
        ),
        Motor.fromSparkMax(
            5, 
            false, 
            spark -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
                config
                    .encoder
                    .positionConversionFactor(1/6.75 )
                    .velocityConversionFactor(1/6.75 / 60);
                spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ),
            Optional.of(new FeedforwardConstants(0, 0.81751, 0.81766, 0.81766)),
            TargetType.Position
        ),
    }; 
    public SwerveTest() {
        HoundLog.setEnabled(true);
        HoundLog.setOptions(new DogLogOptions(() -> !DriverStation.isFMSAttached(), true, true, true, true, 1000));
        SysIDCommands commands = identifyAngleMotors();
        SmartDashboard.putData("Dynamic Forward", commands.dynamicForward());
        SmartDashboard.putData("Dynamic Reverse", commands.dynamicReverse());
        SmartDashboard.putData("Quasistatic Forward", commands.quasistaticForward());
        SmartDashboard.putData("Quasistatic Reverse", commands.quasistaticReverse());
    }

    public SysIDCommands identifyDriveMotors() {
        return driveMotors[0].getSynchronizedSysIDCommands(
            "DriveMotors", 
            0.5, 
            5, 
            10, 
            driveMotors[1], driveMotors[2], driveMotors[3]
        );
    }

    public SysIDCommands identifyAngleMotors() {
        return angleMotors[0].getSynchronizedSysIDCommands(
            "AngleMotors", 
            1, 
            10, 
            10, 
            angleMotors[1], angleMotors[2], angleMotors[3]
        );
    }

    public Command tuneDriveMotors(XboxController xbox) {
        return Commands.run(() -> {
            double target = -xbox.getLeftY() * 4;
            for (Motor motor : driveMotors) {
                motor.setTarget(target);
            }
        }).finallyDo(
            () -> {
                for (Motor motor : driveMotors) {
                    motor.setVoltage(0);
                }
            }
        );
    }

    public Command tuneAngleMotors(XboxController xbox) {
        return Commands.run(() -> {
            double target = Math.atan2(-xbox.getLeftY(), -xbox.getLeftX());
            target = MathUtil.inputModulus(target / (2 * Math.PI), 0, 1);
            for (Motor motor : angleMotors) {
                motor.setTarget(target);
            }
        }).finallyDo(
            () -> {
                for (Motor motor : angleMotors) {
                    motor.setVoltage(0);
                }
            }
        );
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        HoundLog.log("Drive Motors", driveMotors);
        HoundLog.log("Angle Motors", angleMotors);
    }
}

