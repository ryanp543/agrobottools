/*!
 *  file   p2Enumerations.java
 *
 *  date   Aug 19, 2014
 *
 *  author Ahmad Fadeel
 *
 *  brief  This class has the enumerations defined for the specAppManager package
 *
 *  copyright
 *  Copyright (c) 2004-2014. Si-Ware Systems. All Rights Reserved.
 *
 *                PROPRIETARY INFORMATION
 *
 *  This file is CONFIDENTIAL and PROPRIETARY and is for the use of
 *  Si-Ware Systems personnel only. Permission to use it otherwise
 *  must be explicitly obtained from Si-Ware Systems.
 */
package sws.p3AppManager_micro.utils;

public class p3Enumerations {
	
	/*
	 * ! This enum is responsible for Types of the Restore to default Function
	 */
	public enum RestoreOptionsEnum {
		OPTICAL_GAIN_SETTINGS(0, "Optical gain settings"),
		CORRECTION_SETTINGS(1, "Correction settings"),
		ALL(3, "All"); 

		private final int numVal;
		private final String stringVal;

		RestoreOptionsEnum(int numVal, String stringVal) {
			this.numVal = numVal;
			this.stringVal = stringVal;
		}

		public int getNumVal() {
			return numVal;
		}
		public String getStringVal() {
			return stringVal;
		}
	}
	
	/*
	 * ! This enum is responsible for Type of graph you want to plot either
	 * corrected or uncorrected.
	 */
	public enum p3CorrectionType {
		Corrected(0), /* !< corrected graph */
		Uncorrected(1); /* !< uncorrected graph */

		private final int numVal;

		p3CorrectionType(int numVal) {
			this.numVal = numVal;
		}

		public int getNumVal() {
			return numVal;
		}
	}

	/***************************************************************** app manager enum **********************************************************************/
	/*
	 * ! This enum is responsible for the state of the application manager (only
	 * can be changed through AppManager_changeState function).
	 */
	public enum p3AppManagerState {
		Idle(0), /* !< Idle */
		Initialize(1, p3DeviceAction.initializeCore), /* !< Initialize */
		InterferogramRun(2, p3DeviceAction.RunInterSpec), /* !< Interferogram Run */
		SpectroscopySampleRun(3, p3DeviceAction.RunSpecSample), /* !< Spectroscopy Sample Run */
		SpectroscopyBackgroundRun(4, p3DeviceAction.RunSpecBackground), /* ! < Spectroscopy Background Run */
		ActuationProfileSetting(5, p3DeviceAction.SetActuation), /* ! < Actuation Profile Setting */
		CheckingDeviceStatus(6, p3DeviceAction.CheckBoardStatus), /* ! < Checking Device Status */
		CapCurrentRun(7, p3DeviceAction.RunCapCurrent), /* ! < cap current run */
		CalibrationCapVsTimeRun(8, p3DeviceAction.RunCapTimeCalibration), /* ! < Calibration Cap v.s. Time Run */
		CalibrationDelayCompensationRun(9, p3DeviceAction.RunDelayCompensation), /* ! < Calibration Delay Compensation Run */
		CalibrationCoreRun(10, p3DeviceAction.RunCalibration), /* ! < Calibration core run (laser / WL / Meth.) */
		CalibrationGeneration(11, p3DeviceAction.GenerateCalibration), /* ! < Calibration generation */
		SampleIDBurn(12, p3DeviceAction.BurnSampleID), /* ! < SampleID Burn */
		SampleFoldersBurn(13, p3DeviceAction.BurnSampleFolders), /* ! < Sample Folders Burn */
		TempReading(14, p3DeviceAction.ReadTemp), /* ! < Temp Reading */
		ASICRegistersReading(15, p3DeviceAction.ReadASICRegisters), /* ! < ASIC Registers Reading */
		ASICRegistersWriting(16, p3DeviceAction.WriteASICRegisters), /* ! < ASIC Registers Writing */
		SampleFoldersReading(17, p3DeviceAction.ReadSampleFolders), /* ! < Sample Folders Reading */
		ResponseCalculation(18, p3DeviceAction.CalculateResponse), /* ! < Response Calculation (closed loop) */
		ParametersCalculation(19, p3DeviceAction.CalculateParameters), /* ! < Parameters Calculation (closed loop) */
		CoefficientsTrimming(20, p3DeviceAction.TrimCoefficients), /* ! < Coefficients Trimming (closed loop) */
		PhaseTrimming(21, p3DeviceAction.TrimPhase), /* ! < Phase Trimming (closed loop) */
		StabilityCheck(22, p3DeviceAction.CheckStability), /* ! < Stability Check (closed loop) */
		WaveformPreview(23, p3DeviceAction.RunWaveform), /* ! < Waveform Preview (closed loop) */
		gainAdjustInterSpecRun(24, p3DeviceAction.RunGainAdjustInterSpec), /* !< Adaptive Gain Run */
		SNR_Run(25, p3DeviceAction.RunSNR), /* !< SNR Run */
		selfCorr_Run(26, p3DeviceAction.RunSelfCorr), /* !< Interferogram gain and offset correction calculation */
		wavelengthCalibration_Run(27, p3DeviceAction.RunWavelengthCalibration), /* !< Wavelength calibration in spectrum tab */
		StabilityRun(28, p3DeviceAction.RunStability), /* !< Stability Run */
		gainAdjustSpecBG_Run(29, p3DeviceAction.RunGainAdjustSpecBG), /* !< Adaptive Gain BG Run */
		gainAdjustSpecSampleRun(30, p3DeviceAction.RunGainAdjustSpecSample), /* !< Adaptive Gain Sample Run */
		wavelengthCalibrationBG_Run(31, p3DeviceAction.RunWavelengthCalibrationBG), /* !< BG Measurement for Wavelength calibration in spectrum tab */
		updateFFT_SettingsInterSpecRun(32, p3DeviceAction.RunUpdateFFT_SettingsInterSpec), /* !< BG Measurement for Wavelength calibration in spectrum tab */
		updateFFT_SettingsSpecRun(33, p3DeviceAction.RunUpdateFFT_SettingsSpec), /* !< BG Measurement for Wavelength calibration in spectrum tab */
		RestoreDefault(34, p3DeviceAction.RestoreDefaultSettings), /* ! < Restore Default Settings */
		BurnSettings(35, p3DeviceAction.BurnWorkingSettings), /* ! <Burn Working Settings */
		FastPhaseTrimming(36, p3DeviceAction.TrimPhaseFast), /* ! < Fast Phase Trimming (closed loop) */
		SleepAction(37, p3DeviceAction.SleepAction),
		WakeUpAction(38, p3DeviceAction.WakeUpAction),
		PowerOff(38, p3DeviceAction.PowerOff),
		PowerOn(39, p3DeviceAction.PowerOn),
		;
		private final int numVal;
		private final p3DeviceAction deviceAction;

		p3AppManagerState(int numVal) {
			this.numVal = numVal;
			deviceAction = null;
		}

		p3AppManagerState(int numVal, p3DeviceAction deviceAction) {
			this.numVal = numVal;
			this.deviceAction = deviceAction;
		}

		public int getNumVal() {
			return numVal;
		}

		public p3DeviceAction getDeviceAtion() {
			return this.deviceAction;
		}
	}

	/*
	 * ! This enum is responsible for the substate of the application manager
	 * (only can be changed through AppManager_changeState function).
	 */
	public enum p3DeviceAction {
		initializeCore(0), /* !< initialize Core Action */
		RunInterSpec(1), /* !< Run Inter Spec */
		RunSpecBackground(2), /* !< Run Spec background */
		RunSpecSample(3), /* !< Run Spec sample */
		SetActuation(4), /* !< Set Actuation profile */
		CheckBoardStatus(5), /* !< Check Board Status */
		RunCapCurrent(6), /* !< Check Board Status */
		RunCapTimeCalibration(7), /* !< Run Cap vs. Time calibration */
		RunDelayCompensation(8), /* !< Run Delay compensation calibration */
		RunCalibration(9), /* !< Run Calibration core (WL / laser / meth.) */
		GenerateCalibration(10), /* !< generate Calibration */
		BurnSampleID(11), /* !< Burn SampleID */
		BurnSampleFolders(12), /* !< Burn Sample Folders */
		ReadTemp(13), /* !< Read Temp */
		ReadASICRegisters(14), /* !< Read ASIC Registers */
		WriteASICRegisters(15), /* !< Write ASIC Registers */
		ReadSampleFolders(16), /* !< Read Sample Folders */
		CalculateResponse(17), /* !< Calculate Response (closed loop) */
		CalculateParameters(18), /* !< Calculate Parameters (closed loop) */
		TrimCoefficients(19), /* !< Trim Coefficients (closed loop) */
		TrimPhase(20), /* !< Trim Phase (closed loop) */
		CheckStability(21), /* !< Check Stability (closed loop) */
		RunWaveform(22), /* !< Run Waveform (closed loop) */
		RunGainAdjustInterSpec(23), /* !< Run Gain Adjustment for InterSpec Panel */
		RunSNR(24), /* !< Run SNR */
		RunSelfCorr(25), /* !< Run Interferogram gain and offset correction calculation */
		RunWavelengthCalibration(26), /* !< Wavelength calibration in spectrum tab */
		RunStability(27), /* !< Stability Run */
		RunGainAdjustSpecBG(28), /* !< Run BG Gain Adjustment for Spec Panel */
		RunGainAdjustSpecSample(29), /* !< Run Sample Gain Adjustment for Spec Panel */
		RunWavelengthCalibrationBG(30), /* !< BG Measurement for wavelength calibration in Spec Panel */
		RunUpdateFFT_SettingsInterSpec(31), /* !< Updating graphs after changing FFT settings in Inter Spec Panel */
		RunUpdateFFT_SettingsSpec(32), /* !< Updating graphs after changing FFT settings in Spec Panel */
		RestoreDefaultSettings(33), /* !< Restore Default Settings */
		BurnWorkingSettings(34),/* !< Burn Working Settings */
		TrimPhaseFast(35), /* !< Trim Phase Fast (closed loop) */
		SleepAction(36),
		WakeUpAction(37),
		PowerOff(38),
		PowerOn(39),
		;
		private final int numVal;

		p3DeviceAction(int numVal) {
			this.numVal = numVal;
		}

		public int getNumVal() {
			return numVal;
		}
	}

	/*
	 * ! This enum is responsible for application manager error codes (result of
	 * any function in application manager).
	 */
	public enum p3AppManagerStatus {
		NO_ERROR(0), /* !< No Error */
		DEVICE_BUSY_ERROR(1), /* !< Device is busy */
		BOARD_DISTCONNECTED_ERROR(2), /* !< Board disconnected error */
		BOARD_NOT_INITIALIZED_ERROR(3), /* !< Board not initialized error*/
		UNKNOWN_ERROR(4), /* !< Unknown error*/
		CONFIG_FILES_LOADING_ERROR(7), /* !< Error occurred during loading configuration files */
		CONFIG_PARAM_LENGTH_ERROR(8), /* !< Configuration Files Lengths Mismatch */
		INVALID_RUN_TIME_ERROR(11), /* !< Run Time Error */ 
		INAVLID_REG_FILE_FORMAT_ERROR(23), /* !< Register File Incorrect Format */
		NO_OF_SCANS_DSP_ERROR(24), /* !< DSP error*/
		DSP_INTERFEROGRAM_POST_PROCESSING_ERROR(25), /* !< DSP error*/
		DSP_INTERFEROGRAM_POST_EMPTY_DATA_ERROR(26), /* !< DSP error*/
		DSP_INTERFEROGRAM_POST_BAD_DATA_ERROR(27),  /* !< DSP error*/
		UPDATE_CORR_FILE_ERROR(28),  /* Error during updating resolution folder*/
		WHITE_LIGHT_PROCESSING_ERROR(29), /* !< Error in saving background data */
		DSP_INTERFEROGRAM_FFT_POST_PROCESSINF_ERROR(30), /* !< DSP error*/
		INVALID_RUN_PARAMETERS_ERROR(31), /* !< Invalid run parameters */
		INVALID_RUN_TIME_NOT_EQUAL_BG_RUN_TIME_ERROR(32), /* !< run time should be equal to last background time */
		NO_VALID_BG_DATA_ERROR(33), /* !< no valid background data */
		INTERFERO_FILE_CREATION_ERROR(34), /* !< Error while trying to create interferogram file */
		PSD_FILE_CREATION_ERROR(35), /* !< Error while trying to create PSD file */
		SPECTRUM_FILE_CREATION_ERROR(36), /* !< Error while trying to create spectrum file */
		GRAPHS_FOLDER_CREATION_ERROR(37), /* !< Error occurred during creating data folder*/
		INVALID_APODIZATION_WINDOW(38), /* !< Error occurred while loading an invalid apodization window number*/
		INITIATE_TAIFDRIVER_ERROR(42),  /* !< Error occured while NeoSpectra module initialization */
		INVALID_BOARD_CONFIGURATION_ERROR(43), /* !< Error occurred during NeoSpectra module initialization */
		DATA_STREAMING_TAIF_ERROR(50), /* !< Error occurred during streaming from NeoSpectra module */
		DATA_STREAMING_ERROR(51), /* !< Error occurred during streaming from NeoSpectra module */
		INVALID_NOTIFICATION_ERROR(52), /* !< Error occurred during result return */
		INVALID_ACTION_ERROR(53), /* !< Invalid action performed */
		INVALID_DEVICE_ERROR(54), /* !< Invalid device is attached */
		THREADING_ERROR(55), /* !< Threading error occurred */
		BOARD_ALREADY_INITIALIZED(56), /* !< Notification that initialization is already done*/
		INITIALIZATION_IN_PROGRESS (57), /* !< Initialization is in progress*/
		ACTUATION_SETTING_ERROR(60), /* !< Error occurred during the setup of actuation settings */
		DEVICE_IS_TURNED_OFF_ERROR(61), /* !< NeoSpectra module is switched off*/
		ASIC_REGISTER_WRITING_ERROR(62),  /* !< Error occurred during writing to chip registers */
		WHITE_FILE_NOT_EXIST_ERROR(76),   /* !< White light reading not exist*/
		FAILED_IN_ADAPTIVE_GAIN(110), /* !< Adaptive Gain Error */
		ASIC_REGISTER_READING_ERROR(111), /* !< ASIC registers reading error */
		TWO_POINTS_CORR_CALIB_FOLDER_ERROR(112), /* !< ASIC registers reading error */
		FAILED_TO_WRITE_OPTICAL_OPTION_TO_FILE(113),  /* !< Failed to write optical option to file */
		FAILED_TO_CREATE_OPTICAL_SETTINGS_FILE(114),  /* !< Failed to create optical settings file */
		STANDARD_CALIBRATOR_FILE_NOT_EXIST_ERROR(115),  /* !< Failed to load standard calibrator file */
		WAVELENGTH_CALIBRATION_ERROR(116),  /* !< ERROR in wavelength calibration DSP*/
		NO_VALID_OLD_MEASUREMENT_ERROR(117),  /* !< No valid old measurement*/
		DSP_UPDATE_FFT_SETTINGS_ERROR(118),  /* !< Error while updating results after changing FFT settings*/
		GAIN_OPTIONS_ERROR(119), /* !< Fail to find gain option in savedOpticalSettings*/
		
		
		USBCommunicationTimeOutError (199), /* !< Error occurred  during USB communication*/
		CommunicationWriteError (201), /* !< Error occurred during TAIF writing register */
		CommunicationReadError (202),  /* !< Error occurred during TAIF reading register*/
		FLASHING_CONFIGURATION_ERROR (203), /* !< Error occurred during flash the program*/
		P3_SPI_TAIF_RX_OR_ERROR (204), /* !< Receive overrun flag (asserted when new operation is started before the previous data received from single access operation is read, cleared by reading this register)*/
		P3_SPI_TAIF_IN_ADDR_ERROR (205), /* !< Memory Address pointer is out of accepted range*/
		SCAN_TIME_LIMIT_ERROR(212),	/* !< Scan time range is outside acceptable limit*/
		ROM_INVALID_ID (213), /* !< sample ID isn't correct*/
		DEVICE_NOT_INITIALIZED_ERROR (214), /* !< sample ID isn't correct*/
		DEVICE_IS_BUSY_ERROR (215),
		INITIALIZATION_IS_IN_PROGRESS (216),
		SAMPLE_ID_INVALID_ERROR (217),
		SAMPLE_FOLDERS_INVALID_ERROR (218),	/* sample folders data isn't correct */
		OPTICAL_FILE_ERROR (228),			/* optical settings file error */
		NOT_ENOUGH_MEMORY_ERROR (229),		/* not enough memory error */

		I2_STAT_INT1_END_TIMEOUT (230),				/* ASIC returned error during interpolation from block1*/
		I2_STAT_INT1_END_INVALID (231),				/* ASIC returned error during interpolation from block1*/
		I2_STAT_INT1_AVG_OVERFLOW (232),				/* ASIC returned error during interpolation from block1*/
		I2_STAT_INT1_CORE_INVALID_REGION (233),		/* ASIC returned error during interpolation from block1*/
		I2_STAT_INT1_CORE_TIMEOUT (234),				/* ASIC returned error during interpolation from block1*/
		I2_STAT_INT1_CORE_OVERFLOW (235),			/* ASIC returned error during interpolation from block1*/
		I2_STAT_INT1_START_TIMEOUT (236),			/* ASIC returned error during interpolation from block1*/

		I2_STAT_INT2_END_TIMEOUT (237),				/* ASIC returned error during interpolation from block2*/
		I2_STAT_INT2_END_INVALID (238),				/* ASIC returned error during interpolation from block2*/
		I2_STAT_INT2_AVG_OVERFLOW (239),				/* ASIC returned error during interpolation from block2*/
		I2_STAT_INT2_CORE_INVALID_REGION (240),		/* ASIC returned error during interpolation from block2*/
		I2_STAT_INT2_CORE_TIMEOUT (241),				/* ASIC returned error during interpolation from block2*/
		I2_STAT_INT2_CORE_OVERFLOW (242),			/* ASIC returned error during interpolation from block2*/
		I2_STAT_INT2_START_TIMEOUT (243),			/* ASIC returned error during interpolation from block2*/


		INVALID_SAMPLE_FOLDER_VERSION (244),	/*Version number of sample folder isn't supported*/
		TAIF_STREAMING_ERROR_INT1 (245),	/*Error while calculating the working section offset1*/
		STREAMING_TIMEOUT_ERROR (246),		/*Error due to timeout of the streaming interpolation data*/
		TAIF_STREAMING_ERROR_INT2 (247), /*Error while calculating the working section offset2*/

		/************************* FFT ERRORs***************************/
		P3_FFT_ADDRESS_ERROR (248),  	 /* !< */
		
		/************************* Flash ERRORs***************************/
		CRC_NOT_MATCHED (249),
		PATTERN_NOT_MATCHED (250),
		FLASH_FAILED (251),
		IN_ADDRESS_ERROR (252), /* !< */
		RX_OR_ERROR (253),      /* !< */
		WRITE_ENABLE_FAILED (254),
		WRITE_DISABLE_FAILED (255),
		FLASH_BUSY_ERROR (256), /* !< */
		/************************* SPI TAIF ERRORs***************************/
		P3_SPI_TAIF_ADDRESS_ERROR (259), /* !< Error in TAIF Register address to be written or read*/
		
		/************************* FIR ERRORs***************************/
		P3_FIR_ADDRESS_ERROR (260),  /* !< Invalid address*/
		P3_FIR_INVALID_ADD_DATA_ERROR (261), /* !< Error flag when addresses of input data and output data are not in range of assigned memory for filter 1--> invalid*/
		P3_FIR_INVALID_SAMPLES_NUMBER_ERROR (262), /* !< Error flag when number of samples less than number of taps, operation will not start until number of samples >= number of taps, 1--> invalid*/
		P3_FIR_INVALID_ADD_COEFF_ERROR (263), /* !< Error flag when addresses of coeff  are not in range of assigned memory for filter 1--> invalid*/
		P3_FIR_ACC1_SAT_ERROR (264), /* !< Saturation flag for accumulator 1  , 1 : Saturation*/
		P3_FIR_ACC2_SAT_ERROR (265), /* !< Saturation flag for accumulator 2  , 1 : Saturation*/
		P3_FIR_ACC3_SAT_ERROR (266), /* !< Saturation flag for accumulator 3  , 1 : Saturation*/
		P3_FIR_ACC4_SAT_ERROR (267), /* !< Saturation flag for accumulator 4  , 1 : Saturation*/

		/************************* LIN INTERP ERRORs***************************/
		P3_LIN_INTRP_XNEW_ACC_SAT_ERROR (268),   /* !< Error indicates the saturation of the accumulated Xnew generated internally*/
		P3_LIN_INTRP_XNEW_THRES_SAT_ERROR (269), /* !< Error indicates the saturation of Xnew generated internally as being equal to or exceeding the saturation threshold */
		P3_LIN_INTRP_XNEW_LD_MEM_NON_MON_ERROR (270), /* !< Error indicates that the Xnew loaded from memory isn’t increasing/decreasing in a monotonic way */
		P3_LIN_INTRP_XNEW_OUT_STRTXOLD_RNG_ERROR (271), /* !< Error indicates that Xold(i)>Xnew and Xold(i+1)>Xnew*/
		P3_LIN_INTRP_XNEW_OUT_FNLXOLD_RANGE_ERROR (272), /* !< Error indicates that no more Xold data to be loaded while Xold(i)<Xnew and Xold(i+1)<Xnew*/
		P3_LIN_INTRP_XOLD_NON_MONO_ERROR (273), /* !< Error indicates that Xold isn’t increasing/decreasing in a monotonic way*/
		P3_LIN_INTRP_ZERO_DIV_ZERO_ERROR (274), /* !< Error indicates dividing zero by zero which means Xold(i+1)=Xold(i) = xnew*/
		P3_LIN_INTRP_SCALR_DIV_ZERO_ERROR (275), /* !< Error indicates divide by zero in scalar division mode*/
		P3_LIN_INTRP_WR_XNEW_ERR_ERROR (276), /* !< Error indicates Flag xnew is gated from being written to the memory as its length exceeds 32 bit*/
		P3_LIN_INTRP_DMA_ADDR_WRD_ALGN_ERROR (277), /* !< Error indicates that one of the given addresses isn’t word aligned (the least 2 LSB /= 0)*/
		P3_LIN_INTRP_DMA_ADDR_LSB_IN_RNG_ERROR (278), /* !< Error indicates LSB of one of given addresses is out of the given address space for the HW Accelerator(greater than or equal x5800)*/
		P3_LIN_INTRP_DMA_ADDR_MSB_IN_RNG_ERROR (279), /* !< Error indicates MSB of one of given addresses is out of the given address space for the HW Accelerator (not equal x200)*/
		ACTION_ABORTED (280),  /* !< Error occurred during ISR abort operation*/
		USERINTERFACE_DMA_WRITE_ERROR (281), /* !< Error occurred during DMA write operation*/
		USERINTERFACE_WRONG_OPERATION (282), /* !< Error occurred during read a wrong operation*/
		WDT_WRITE_LOCK_FAILED (283), /* !< Error occurred during write lock*/
		WDT_WRITE_UNLOCK_FAILED (284), /* !< Error occurred during write unlock*/

		/************************* DSP ERRORs***************************/
		DSP_INITIALIZATION_CONFIGURATION_FILES_IS_EMPTY_ERROR (285),  /* !< Error occurred during DSP missing configuration data*/
		DSP_INITIALIZATION_CONFIGURATION_FILES_LENGTH_NOT_VALID_ERROR (286), /* !< Error occurred during DSP initialization configuration length is not valid*/
		DSP_INITIALIZATION_INVALID_INTERFEROGRAM_TYPE_ERROR (287), /* !< Error occurred during DSP initialization for invalid interferogram type*/
		DSP_INTERPOLATION_LINEAR_INPUT_SIZE_ZERO_ERROR (288), /* !< Error occurred during DSP interpolation step streaming input size is zero*/
		DSP_INTERPOLATION_LINEAR_OUTPUT_SIZE_ZERO_ERROR (289), /* !< Error occurred during DSP interpolation step streaming output size is zero*/
		DSP_INTERPOLATION_LINEAR_DIVISION_BY_ZERO_ERROR (290), /* !< Error occurred during DSP interpolation step division by ZERO */
		DSP_MATH_DIVISION_BY_ZERO_ERROR (291), /* !< Error occurred during DSP mathematical division by ZERO operation */
		DSP_Spline_NO_POINTS_ERROR (292), /* !< Error occurred during DSP spline function no of points is not correct */
		DSP_SPLINE_KNOTS_DECREASING_ERROR (293), /* !< Error during DSP Spline cubic operation*/
		DSP_SPLINE_UNKNOWN_ERROR (294), /* !< Error occurred during DSP spline for unknown reason*/
		DSP_FFT_NO_POINTS_ERROR (295), /* !< Error occurred during DSP FFT number of points is not correct*/
		DSP_NOISE_LEVEL_ERROR (296),  /* !< Error occurred during DSP noise level problem*/
		NO_OF_RUNS_LIMIT_ERROR (297),
		USER_COMMUNICATION_TIMEOUT_ERROR (298),
		FFT_WRONG_NUMBER_POINTS (300),
		
		PI_UNKNOWN_COMMAND (301), //Just for reservation
		
		/************************* whiteLightCorrection ERRORs*********/
		WHITE_LIGHT_CORR_MAIN_BURST_PKPK_ERROR (302),
		WHITE_LIGHT_CORR_COUPLING_TONE_DETECTED_ERROR (303),
		WHITE_LIGHT_CORR_BURSTS_RATIO_ERROR (304),
		WHITE_LIGHT_CORR_LASER_ERROR (305),

		;

		private final int numVal;

		p3AppManagerStatus(int numVal) {
			this.numVal = numVal;
		}

		public int getNumVal() {
			return this.numVal;
		}

		// this method used to get the enum by its value
		public static p3AppManagerStatus getAppManagerStatusByCode(int code) {
			for (p3AppManagerStatus appManagerStatus : p3AppManagerStatus.values()) {
				if (appManagerStatus.getNumVal() == code)
					return appManagerStatus;
			}

			return p3AppManagerStatus.UNKNOWN_ERROR;
		}

	}
}
