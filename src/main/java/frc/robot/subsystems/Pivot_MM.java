// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Instrum;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class Pivot_MM extends SubsystemBase {
	
	private double GEAR_RATIO = 80;
	private double MOTOR_COUNTS_PER_REV = 2048;
	final double maxSetpoint_deg = 75.0;
	final double minSetpoint_deg = 0.0;
	private double ForwardSoftLimitThreshold = deg_To_Raw_Sensor_Counts(maxSetpoint_deg);
	private double ReverseSoftLimitThreshold = deg_To_Raw_Sensor_Counts(minSetpoint_deg);



	// private AnalogInput anglePotentiometer;
	// private AnglePotentiometer angleSensor;
	// private double currentPotAngle;
	
	

	// private double maxSetpoint_raw = deg_To_Raw_Sensor_Counts(maxSetpoint_deg);
	// private double minSetpoint_raw = deg_To_Raw_Sensor_Counts(minSetpoint_deg);
	
	WPI_TalonFX _talon = new WPI_TalonFX(Constants.PIVOT_MOTOR, "rio"); // Rename "rio" to match the CANivore device name if using a
													// CANivore




	/* Used to build string throughout loop */
	//StringBuilder _sb = new StringBuilder();

	/** Creates a new Pivot_MM. */
	public Pivot_MM() {
		/* Factory default hardware to prevent unexpected behavior */
		// anglePotentiometer = new AnalogInput(Constants.POT_PORT);
		// anglePotentiometer.setAverageBits(4);
		// angleSensor = new AnalogPotentiometer(anglePotentiometer, 300, 20);
		// currentPotAngle = angleSensor.get();
		
		_talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/*
		 * set deadband to super small 0.001 (0.1 %).
		 * The default deadband is 0.04 (4 %)
		 */
		_talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(false);
		_talon.setInverted(true);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is
		 * integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
		 * sensor-phase
		 */
		// _talon.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		_talon.config_kF(Constants.kSlotIdx, 0.04, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kSlotIdx, 0.058, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kSlotIdx, 0.0, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kSlotIdx, 0.0, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(12000, Constants.kTimeoutMs);
		_talon.configMotionAcceleration(6000, Constants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		_talon.configForwardSoftLimitThreshold(ForwardSoftLimitThreshold, Constants.kTimeoutMs);
		_talon.configForwardSoftLimitEnable(true, 0);
		
		_talon.configReverseSoftLimitThreshold(ReverseSoftLimitThreshold, Constants.kTimeoutMs);
		_talon.configReverseSoftLimitEnable(true, 0);
		// _talon.configClearPositionOnLimitR(true, 0);


		
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("Current Deg", my_getDeg());
		// SmartDashboard.putNumber("pivot encoder", _talon.getSelectedSensorPosition());
		SmartDashboard.putNumber("Pivot Sensor [counts]",  _talon.getSelectedSensorPosition());
		SmartDashboard.putNumber("Pivot Angle [deg]", my_getDeg());
		// currentPotAngle = angleSensor.get();
		// SmartDashboard.putNumber("Pot Angle", currentPotAngle);
		// double motorOutput = _talon.getMotorOutputPercent();

		/* Prepare line to print */
		// _sb.append("\tOut%:");
		// _sb.append(motorOutput);
		// _sb.append("\tVel:");
		// _sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
		// // This method will be called once per scheduler run
		// /* Append more signals to print when in speed mode */
		// _sb.append("\terr:");
		// _sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx));
		// _sb.append("\ttrg:");
		// _sb.append(m_targetPos);

		/* Instrumentation */
		//Instrum.Process(_talon, _sb);
	}

	double m_targetPos = 0.0;

	/**
	 * Input in deg
	 * 
	 * @param deg
	 */
	private double clampSetpointValue_Deg(double value)  {
		double adjustedSetpoint = value;
		if (value > maxSetpoint_deg)  {
			adjustedSetpoint =  maxSetpoint_deg;
		}  else if (value < minSetpoint_deg)  {
			adjustedSetpoint =  minSetpoint_deg;
		}
		return adjustedSetpoint;
	}
	
	
	 public void my_motionMagic_Run(double deg) {
		/* Motion Magic */
		deg = clampSetpointValue_Deg(deg);
		m_targetPos = deg;

		if (m_targetPos > maxSetpoint_deg) {
			m_targetPos = maxSetpoint_deg;
		  } else if (m_targetPos < minSetpoint_deg)  {
			m_targetPos = minSetpoint_deg;
		  }

		/* 2048 ticks/rev * 10 Rotations in either direction */
		double targetPos = deg_To_Raw_Sensor_Counts(m_targetPos);// / 360 * MOTOR_COUNTS_PER_REV * GEAR_RATIO;
		_talon.set(TalonFXControlMode.MotionMagic, targetPos);

	}

	// public void my_Arm_Stop() {
	// 	_talon.set(ControlMode.PercentOutput, 0.0);
	// }


	/**
	 * input -1 to 1
	 * 
	 * @param speed
	 */
	public void my_PercentOutput_Run(double speed) {
		_talon.set(TalonFXControlMode.PercentOutput, speed);
	}

	public void my_resetEncoder() {
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}

	public double my_getDeg() {
		return raw_Sensor_Counts_To_Deg(_talon.getSelectedSensorPosition());// * 360 / (GEAR_RATIO * MOTOR_COUNTS_PER_REV);
	}

	



	/**
	 * Test if in position
	 * 
	 * @param setpoint // in deg
	 * @return
	 */
	public boolean my_get_PositionLock(double setpoint) {
		double positionLockTollerence = 1;
		double error = my_getDeg() - setpoint;

		if (Math.abs(error) <= positionLockTollerence) {
			return true;
		} else {
			return false;
		}
	}

	private double raw_Sensor_Counts_To_Deg(double counts){
		return counts * 360 / (GEAR_RATIO * MOTOR_COUNTS_PER_REV);

	}

	private double deg_To_Raw_Sensor_Counts(double deg){
		return deg / 360 * MOTOR_COUNTS_PER_REV * GEAR_RATIO;
	}


	public double my_getCurrentPivotAngle()  {
		final double m_sensorPos = _talon.getSelectedSensorPosition(0);
		final double m_angle = raw_Sensor_Counts_To_Deg(m_sensorPos);
		SmartDashboard.putNumber("Pivot Sensor [counts]", m_sensorPos);
		SmartDashboard.putNumber("Pivot Angle [deg]", m_angle);
		return m_angle;
	}

	public  double get_My_CurrentRAW_Position() {
		return _talon.getSelectedSensorPosition(0);
	}

	public void set_pivotToCurrent()  {
		_talon.set(ControlMode.Position, get_My_CurrentRAW_Position());
	}

	public double get_MaxAngle()  { 
		return maxSetpoint_deg;
	}

	public double get_minAngle()  {
		return minSetpoint_deg;
	}


}