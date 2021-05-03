package sws.p3AppManager_micro.devices;


import java.io.File;

import org.apache.log4j.Logger;



import sws.p3AppManager_micro.P3Driver;
import sws.p3AppManager_micro.configuration.p3RunConfig;
import sws.p3AppManager_micro.jobs.p3AbstractJob;
import sws.p3AppManager_micro.jobs.p3JobManager;
import sws.p3AppManager_micro.utils.p3AppManagerException;
import sws.p3AppManager_micro.utils.p3AppManagerUtils;
import sws.p3AppManager_micro.utils.p3Constants;
import sws.p3AppManager_micro.utils.p3DeviceNotificationResult;
import sws.p3AppManager_micro.utils.p3Enumerations;
import sws.p3AppManager_micro.utils.p3Enumerations.p3AppManagerState;
import sws.p3AppManager_micro.utils.p3Enumerations.p3AppManagerStatus;
import sws.p3AppManager_micro.utils.p3Enumerations.p3DeviceAction;


public class p3Sample extends p3Device {
	private static Logger logger = Logger.getLogger(p3Sample.class);
	private String sampleId = "";

	private p3AppManagerState state;
	private double[][] SpecResult = null;
	private double[][] bgSpecResult = null;
	private double[][] bgWL_CalibResult = null;
	
	private long[] externalApodizationWindow = null;

	private double[][] GainAdjustSpec_Result = null;

	private double[][] interSpecResult = null;

	private double[][] rawData = null;

	
	private double lastBGTime = 0;

	
	private double gainAdjustError = 1;

	private String productId = "21845"; // 0x5555
	private P3Driver driver;

	// param.conf file (initially standard param.conf and gets updated after
	// every function in DSP during calibration)
	
	// PGA1 and PGA2 multiplication combinations
	// 2 D dimensional array with columns: {PGA1_index, PGA2_index, gain
	// multiplication value}
	// Gain 1 should be higher than Gain 2 in order to reduce the noise
	int[][] PGAs_values = new int[][] { { 0, 0, 1 }, { 1, 0, 2 }, { 1, 1, 4 }, { 3, 0, 6 }, { 2, 1, 8 }, { 5, 0, 10 },
			{ 3, 1, 12 }, { 2, 2, 16 }, { 5, 1, 20 }, { 3, 2, 24 }, { 4, 2, 32 }, { 3, 3, 36 }, { 5, 2, 40 },
			{ 4, 3, 48 }, { 5, 3, 60 }, { 4, 4, 64 }, { 5, 4, 80 }, { 5, 5, 100 } };

	

	public p3Sample() {

		// initiate status by idle state
		state = p3AppManagerState.Idle;
		driver = new P3Driver(Integer.parseInt(productId), p3Constants.APPLICATION_WORKING_DIRECTORY);
	}

	private p3AppManagerState getState() {
		return state;
	}

	private void setState(p3AppManagerState state) {
		logger.info("------State Change------From------" + this.state + "-----To-----" + state);
		this.state = state;
	}

	public String getDeviceId() {

		return sampleId;
	}

	@Override
	/*
	 * ! This function is an API for initialization the class and the board.
	 * 
	 * @param [IN] globalConfig global configuration object
	 * 
	 * @return p3AppManagerStatus status of the operation
	 */
	public p3AppManagerStatus initializeCore(String... params) {

		if (p3AppManagerState.Idle != this.getState()) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}

		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status
				|| p3AppManagerStatus.BOARD_ALREADY_INITIALIZED == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;

		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {
			this.setState(p3AppManagerState.Initialize);
			executeIntiliazation();
		}

		return p3AppManagerStatus.NO_ERROR;
	}

	@Override
	/*
	 * ! This function is an API for checking the status of the connected device
	 * 
	 * @param [IN] globalConfig global configuration object
	 * 
	 * @return p2AppManagerStatus status of the operation
	 */
	public p3AppManagerStatus checkDeviceStatus(String... params) {

		if (p3AppManagerState.Idle != this.getState()) {
			if (p3AppManagerState.Initialize == this.getState()) {
				return p3AppManagerStatus.INITIALIZATION_IN_PROGRESS;
			} else {
				return p3AppManagerStatus.NO_ERROR;
			}
		}

		// set state to run CheckingDeviceStatus
		this.setState(p3AppManagerState.CheckingDeviceStatus);

		p3AppManagerStatus status = checkBoardStatus();

		// return state to idle
		this.setState(p3AppManagerState.Idle);

		logger.info("checkBoardStatus method returned : " + status.toString());

		if (p3AppManagerStatus.BOARD_ALREADY_INITIALIZED == status) {
			return p3AppManagerStatus.NO_ERROR;
		} else if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {
			status = initializeCore();
		}
		// swallow INITIATE_TAIFDRIVER_ERROR error
		else if (status == p3AppManagerStatus.INITIATE_TAIFDRIVER_ERROR) {
			logger.error("INITIATE_TAIFDRIVER_ERROR returned but the application manager swallowed it");
			return p3AppManagerStatus.INITIALIZATION_IN_PROGRESS;
		}

		return status;
	}


	
	@Override
	/*
	 * ! This function is an API for run spectroscopy.
	 * 
	 * @param [IN] globalConfig global configuration object
	 * 
	 * @param [IN] params[0] is the run time in ms
	 * 
	 * @param [IN] params[1] is a boolean value to indicate sample reading or
	 * background reading (true=sample, false=background)
	 * 
	 * @return p2AppManagerStatus status of the operation
	 */
	public p3AppManagerStatus runSpec(String... params) {

		if (params.length != 7 && params.length != 2) {
			return p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR;
		}

		// check that device not busy
		if (p3AppManagerState.Idle != this.getState()) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}

		// check boardStatus
		boolean initialize = false;
		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;
		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {

			initialize = true;
		}

		p3RunConfig runConfig;
		// get run type sample or back ground
		boolean isSample = Boolean.parseBoolean(params[1]);
		try {
			if (isSample) {
				if (this.bgSpecResult == null) {
					return p3AppManagerStatus.NO_VALID_BG_DATA_ERROR;
				}
				if (this.lastBGTime < Double.parseDouble(params[0])) {
					return p3AppManagerStatus.INVALID_RUN_TIME_NOT_EQUAL_BG_RUN_TIME_ERROR;
				}
				runConfig = initiateRunConfig(p3DeviceAction.RunSpecSample, params);
				// reset result value for new run
				this.SpecResult = null;

			} else {
				this.lastBGTime = Double.parseDouble(params[0]);
				runConfig = initiateRunConfig(p3DeviceAction.RunSpecBackground, params);
				// reset result value for new run
				this.bgSpecResult = null;
			}
		} 
		catch(p3AppManagerException ex)
		{
			return p3AppManagerStatus.getAppManagerStatusByCode(ex.getErrorCode());
		}
		catch (Exception ex) {
			return p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR;
		}
		executeRunSpecByRunType(runConfig, initialize);

		return p3AppManagerStatus.NO_ERROR;
	}

	@Override
	/*
	 * ! This function is an API for wavelength calibration run
	 * 
	 * @param [IN] globalConfig global configuration object
	 * 
	 * @param [IN] params[0] is the run time in ms
	 * 
	 * @param [IN] params[1] is apodization option
	 * 
	 * @param [IN] params[2] is zeroPadding option
	 * 
	 * @return p2AppManagerStatus status of the operation
	 */
	public p3AppManagerStatus wavelengthCalibrationBG(String... params) {

		if (params.length != 5) {
			return p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR;
		}

		// check that device not busy
		if (p3AppManagerState.Idle != this.state) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}

		// check boardStatus
		boolean initialize = false;
		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;
		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {

			initialize = true;
		}

		p3RunConfig runConfig;
		try {
			this.bgWL_CalibResult = null;
			runConfig = initiateRunConfig(p3DeviceAction.RunWavelengthCalibrationBG, params);
		} 
		catch(p3AppManagerException ex)
		{
			return p3AppManagerStatus.getAppManagerStatusByCode(ex.getErrorCode());
		}
		catch (Exception ex) {
			return p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR;
		}
		executeRunSpecByRunType(runConfig, initialize);

		return p3AppManagerStatus.NO_ERROR;
	}

	@Override
	/*
	 * ! This function is an API for wavelength calibration run
	 * 
	 * @param [IN] globalConfig global configuration object
	 * 
	 * @param [IN] params[0] is the run time in ms
	 * 
	 * @param [IN] params[1] is a string representing the standard calibrator
	 * used
	 * 
	 * @return p2AppManagerStatus status of the operation
	 */
	public p3AppManagerStatus wavelengthCalibration(String... params) {

		if (params.length != 6) {
			return p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR;
		}

		// check that device not busy
		if (p3AppManagerState.Idle != this.state) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}

		// check boardStatus
		boolean initialize = false;
		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;
		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {

			initialize = true;
		}

		p3RunConfig runConfig;
		try {
			if (this.bgWL_CalibResult == null) {
				return p3AppManagerStatus.NO_VALID_BG_DATA_ERROR;
			}
			runConfig = initiateRunConfig(p3DeviceAction.RunWavelengthCalibration, params);

			double wells[] = p3AppManagerUtils
					.loadRawDataFile(p3Constants.getPath(p3Constants.STANDARD_CALIBRATORS_FOLDER_PATH) + File.separatorChar
							+ runConfig.standardCalibratorType + p3Constants.STANDARD_CALIBRATORS_FILE_EXT);

			runConfig.CalibrationWells = new int[wells.length];
			
			for(int i=0; i< wells.length; i++)
			{
				runConfig.CalibrationWells[i] =  (int)(long)(wells[i] * Math.pow(2, 20));
			}
			
			if (runConfig.CalibrationWells == null) {
				// File was not found
				throw new p3AppManagerException("Failed to load standard calibrator file ",
						p3AppManagerStatus.WHITE_FILE_NOT_EXIST_ERROR.getNumVal());
			}
			
		} 
		catch(p3AppManagerException ex)
		{
			return p3AppManagerStatus.getAppManagerStatusByCode(ex.getErrorCode());
		}
		catch (Exception ex) {
			return p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR;
		}
		
		executeRunSpecByRunType(runConfig, initialize);

		return p3AppManagerStatus.NO_ERROR;
	}

	@Override
	/*
	 * ! This function is an API for getting tranmittance error
	 * 
	 * @return transmittance error
	 */
	public double getErrorData() {
		return this.gainAdjustError;
	}

	@Override
	/*
	 * ! This function is an API for getting spectroscopy data.
	 * 
	 * @return spectroscopy data
	 */
	public double[][] getSpecData() {
		if (p3AppManagerState.Idle != this.getState()) {
			logger.error("p2Device is busy now");
			return null;
		}

		if (SpecResult == null) {
			logger.error("there is no Spec data may be you don't have any successful spec run ");
			return null;
		}

		return SpecResult;
	}

	@Override
	public void setExternalApodizationWindow(long[] apodizationWindow) {
		if(apodizationWindow == null)
		{
			externalApodizationWindow = null;
			return;
		}
		
		externalApodizationWindow = new long[apodizationWindow.length];
		System.arraycopy(apodizationWindow, 0, externalApodizationWindow, 0, apodizationWindow.length);
	}


	@Override
	/*
	 * ! This function is an API for run interferogram and spectrum.
	 * 
	 * @param [IN] globalConfig global configuration object
	 * 
	 * @param [IN] params[0] is the run time in ms
	 * 
	 * @return p2AppManagerStatus status of the operation
	 */
	public p3AppManagerStatus runInterSpec(String... params) {

		if (params.length != 6 && params.length != 1) {
			return p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR;
		}

		// check that device not busy
		if (p3AppManagerState.Idle != this.getState()) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}
		// check boardStatus
		boolean initialize = false;
		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;
		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {

			initialize = true;
		}

		// reset result value for new run
		this.interSpecResult = null;
		p3RunConfig runConfig;

		try {
			runConfig = initiateRunConfig(p3DeviceAction.RunInterSpec, params);
		} 
		catch(p3AppManagerException ex)
		{
			return p3AppManagerStatus.getAppManagerStatusByCode(ex.getErrorCode());
		}
		catch (Exception e) {
			return p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR;
		}

		// set state to run interspectro
		this.setState(p3AppManagerState.InterferogramRun);
		executeRunSpecByRunType(runConfig, initialize);

		return p3AppManagerStatus.NO_ERROR;
	}



	@Override
	/*
	 * ! This function is an API for run Interferogram correction calculation
	 * 
	 * @param [IN] globalConfig global configuration object
	 * 
	 * @param [IN] params[0] is the run time in ms
	 * 
	 * @return p2AppManagerStatus status of the operation
	 */
	public p3AppManagerStatus runCalibCorr(String... params) {

		if (params.length != 5) {
			return p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR;
		}

		// check that device not busy
		if (p3AppManagerState.Idle != this.getState()) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}
		// check boardStatus
		boolean initialize = false;
		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;
		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {

			initialize = true;
		}

		// reset result value for new run
		this.interSpecResult = null;
		p3RunConfig runConfig;

		try {
			runConfig = initiateRunConfig(p3DeviceAction.RunSelfCorr, params);
		} 
		catch(p3AppManagerException ex)
		{
			return p3AppManagerStatus.getAppManagerStatusByCode(ex.getErrorCode());
		}
		catch (Exception e) {
			return p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR;
		}

		// set state to run interspectro
		this.setState(p3AppManagerState.selfCorr_Run);
		executeRunSpecByRunType(runConfig, initialize);

		return p3AppManagerStatus.NO_ERROR;
	}

	@Override
	/*
	 * ! This function is an API for getting raw data as streamed from module.
	 * 
	 * @return raw data
	 */
	public double[][] getRawData() {
		if (p3AppManagerState.Idle != this.getState()) {
			logger.error("p2Device is busy now");
			return null;
		}

		if (rawData == null) {
			logger.error("there is no raw data may be you don't have any successful runs ");
			return null;
		}

		return rawData;
	}

	@Override
	/*
	 * ! This function is an API for getting interferogram and spectrum data.
	 * 
	 * @return interferogram and spectrum data
	 */
	public double[][] getInterSpecData() {
		if (p3AppManagerState.Idle != this.getState()) {
			logger.error("p2Device is busy now");
			return null;
		}

		if (interSpecResult == null) {
			logger.error("there is no interSpec data may be you don't have any successful interspec run ");
			return null;
		}

		return interSpecResult;
	}

	
	@Override
	/*
	 * ! This function is an API for getting gain adjustment data for Spec Panel
	 * 
	 * @return interferogram and spectrum data
	 */
	public double[][] getGainAdjustSpecData() {
		if (p3AppManagerState.Idle != this.getState()) {
			logger.error("p2Device is busy now");
			return null;
		}

		if (GainAdjustSpec_Result == null) {
			logger.error("there is no valid data");
			return null;
		}

		return GainAdjustSpec_Result;
				
	}

	@Override
	public p3AppManagerStatus restoreDefaultSettings(String... params)
	{
		// check that device not busy
		if (p3AppManagerState.Idle != this.getState()) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}
		// check boardStatus
		boolean initialize = false;
		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;
		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {
			initialize = true;
		}

		p3RunConfig runConfig = new p3RunConfig();
		runConfig.runType = p3AppManagerState.RestoreDefault;
		if(params.length != 0)
			runConfig.restoreOption = params[0];
		else
			runConfig.restoreOption = p3Enumerations.RestoreOptionsEnum.ALL.getStringVal();

		// set state to write ASIC registers
		this.setState(p3AppManagerState.RestoreDefault);
		executeRunSpecByRunType(runConfig, initialize);
		return p3AppManagerStatus.NO_ERROR;
	}
	
	@Override
	public p3AppManagerStatus sleepAction(){
		// check that device not busy
		if (p3AppManagerState.Idle != this.getState()) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}
		// check boardStatus
		boolean initialize = false;
		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;
		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {
			initialize = true;
		}
		
		p3RunConfig runConfig = new p3RunConfig();
		runConfig.runType = p3AppManagerState.SleepAction;
		
		// set state to write ASIC registers
		this.setState(p3AppManagerState.SleepAction);
		executeRunSpecByRunType(runConfig, initialize);
		return p3AppManagerStatus.NO_ERROR;
	}
	
	@Override
	public p3AppManagerStatus wakeUpAction(){
		// check that device not busy
		if (p3AppManagerState.Idle != this.getState()) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}
		// check boardStatus
		boolean initialize = false;
		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;
		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {
			initialize = true;
		}
		
		p3RunConfig runConfig = new p3RunConfig();
		runConfig.runType = p3AppManagerState.WakeUpAction;
		
		// set state to write ASIC registers
		this.setState(p3AppManagerState.WakeUpAction);
		executeRunSpecByRunType(runConfig, initialize);
		return p3AppManagerStatus.NO_ERROR;
	}
	
	@Override
	public p3AppManagerStatus powerOff(){
		// check that device not busy
		if (p3AppManagerState.Idle != this.getState()) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}
		// check boardStatus
		boolean initialize = false;
		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;
		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {
			initialize = true;
		}
		
		p3RunConfig runConfig = new p3RunConfig();
		runConfig.runType = p3AppManagerState.PowerOff;
		
		// set state to write ASIC registers
		this.setState(p3AppManagerState.PowerOff);
		executeRunSpecByRunType(runConfig, initialize);
		return p3AppManagerStatus.NO_ERROR;
	}
	
	@Override
	public p3AppManagerStatus powerOn(){
		// check that device not busy
		if (p3AppManagerState.Idle != this.getState()) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}
		// check boardStatus
		boolean initialize = false;
		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;
		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {
			initialize = true;
		}
		
		p3RunConfig runConfig = new p3RunConfig();
		runConfig.runType = p3AppManagerState.PowerOn;
		
		// set state to write ASIC registers
		this.setState(p3AppManagerState.PowerOn);
		executeRunSpecByRunType(runConfig, initialize);
		return p3AppManagerStatus.NO_ERROR;
	}
	
	@Override
	public p3AppManagerStatus burnSettingsSpecificSettings(String... params)
	{
		// check that device not busy
		if (p3AppManagerState.Idle != this.getState()) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}
		// check boardStatus
		boolean initialize = false;
		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;
		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {
			initialize = true;
		}

		p3RunConfig runConfig = new p3RunConfig();
		runConfig.runType = p3AppManagerState.BurnSettings;
		runConfig.burnMode = Integer.parseInt(params[0]);

		// set state to write ASIC registers
		this.setState(p3AppManagerState.BurnSettings);
		executeRunSpecByRunType(runConfig, initialize);
		return p3AppManagerStatus.NO_ERROR;
	}

	@Override
	public String[] getSoftwareVersion()
	{
		return driver.readSWVersion();
	}
	

	@Override
	public p3AppManagerStatus runSpecGainAdjBG(String... params) {	
		
		if (params.length != 1) {
			return p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR;
		}
		
		// check that device not busy
		if (p3AppManagerState.Idle != this.getState()) {
			return p3AppManagerStatus.DEVICE_BUSY_ERROR;
		}
		// check boardStatus
		boolean initialize = false;
		p3AppManagerStatus status = checkBoardStatus();
		if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status || p3AppManagerStatus.UNKNOWN_ERROR == status)
			return status;
		if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {
			initialize = true;
		}
		
		p3RunConfig runConfig;
		
		try {
			runConfig = initiateRunConfig(p3DeviceAction.RunGainAdjustSpecBG, params);
		} 
		catch(p3AppManagerException ex)
		{
			return p3AppManagerStatus.getAppManagerStatusByCode(ex.getErrorCode());
		}
		catch (Exception e) {
			return p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR;
		}
		
		this.setState(p3AppManagerState.gainAdjustSpecBG_Run);
		executeRunSpecByRunType(runConfig, initialize);
		
		return p3AppManagerStatus.NO_ERROR;
	}

	/*
	 * ! This function is responsible execute the thread that running the
	 * initialization core
	 */
	private void executeIntiliazation() {

		try {

					taifDriverInitiation();
					setFinalResultAndnotifyObservers(p3DeviceAction.initializeCore, null, p3AppManagerStatus.NO_ERROR);
				} catch (p3AppManagerException exp) {
					logger.error(exp.getMessage());
					setFinalResultAndnotifyObservers(p3DeviceAction.initializeCore, null,
							p3AppManagerStatus.getAppManagerStatusByCode(exp.getErrorCode()));
				}

	}


	/*
	 * ! This function is responsible execute the thread that apply run command
	 * logic
	 * 
	 * @param [IN] runConfig configuration of run
	 * 
	 * @param [IN] initialize value to indicate whether to initialize the board
	 * before the run or not
	 */
	private void executeRunSpecByRunType(final p3RunConfig runConfig, final boolean initialize) {

		p3JobManager.submit(new p3AbstractJob<Boolean>() {

			@Override
			protected Boolean execute() {

				try {
					double[][] result = null;
					
					// try operation for MAX_ERROR_COUNT times. if it continue failing, then report failure.
					for(int i = 0; i < p3Constants.MAX_ERROR_COUNT;)
					{
						p3RunConfig currentRunConfig = (p3RunConfig) runConfig.clone();
						try{
							result = driver.runOperation(currentRunConfig);
						}
						catch(p3AppManagerException ex)
						{
							//try it for one more time
							i++;
							System.out.println("trial no. "+ i + " failed. error code is " + ex.getErrorCode());
							logger.error("trial no. "+ i + " failed. error code is " + ex.getErrorCode());
							if(i == p3Constants.MAX_ERROR_COUNT)
							{
								throw ex;
							}
							else if(ex.getErrorCode() == 400)
								throw ex;
							else
							{
								continue;
							}
						}
						break;
					}
					if (result == null) {
						throw new p3AppManagerException(
								"No valid result back for this p2Sample : " + sampleId + " run.",
								p3AppManagerStatus.NO_OF_SCANS_DSP_ERROR.getNumVal());
					}
			
					setFinalResultAndnotifyObservers(runConfig.runType.getDeviceAtion(), result,
							p3AppManagerStatus.NO_ERROR);

				} catch (p3AppManagerException exp) {
					logger.error(exp.getMessage());
					setFinalResultAndnotifyObservers(runConfig.runType.getDeviceAtion(), null,
							p3AppManagerStatus.getAppManagerStatusByCode(exp.getErrorCode()));
				}
				catch (Exception e) {
					setFinalResultAndnotifyObservers(runConfig.runType.getDeviceAtion(), null,
							p3AppManagerStatus.DATA_STREAMING_ERROR);
				}
				return true;
			}

		});

	}


	/*************************************************************
	 * Execution Helper methods
	 ************************************************************************/
	/****************************************************
	 * initialization methods
	 *****************************************/

	/*
	 * ! This function is responsible to run taifboard initialization
	 * 
	 * @param [IN] config global configuration of run
	 */
	private void taifDriverInitiation() {

		driver.initiateTAIF();
		try {
			sampleId = driver.readROMID();
		} catch (Exception ex) {
			sampleId = "sampleId";
		}

	}


	/**********************************************************
	 * General Helper methods
	 ************************************************************************/
	/*
	 * ! This function is responsible for checking board status
	 * 
	 * @return p2AppManagerStatus status of the operation
	 */
	private p3AppManagerStatus checkBoardStatus() {

		int boardStatus = driver.checkBoardStatus();

		if (boardStatus == 14)
		{

			return p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR;
		}

		if (boardStatus == 74)
		{
			return p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR;
		}

		return p3AppManagerStatus.BOARD_ALREADY_INITIALIZED;
	}

	/*
	 * ! This function is responsible for initiate run configuration object
	 * 
	 * @param [IN] action Type of the action you want to perform
	 * 
	 * @param [IN] params parameters of the required action (usually received
	 * from the upper layer)
	 * 
	 * @return run configuration object after being initialized
	 */
	private p3RunConfig initiateRunConfig(p3DeviceAction action, String... params)
			throws p3AppManagerException, Exception {
		switch (action) {
		
		case RunInterSpec:
			if (p3AppManagerUtils.isEmptyString(params[0])) {
				throw new p3AppManagerException("Invalid Parameters. Error: ",
						p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR.getNumVal());
			}
			if(params.length == 6 && (p3AppManagerUtils.isEmptyString(params[1])
					|| p3AppManagerUtils.isEmptyString(params[2])
					|| p3AppManagerUtils.isEmptyString(params[3])
					|| p3AppManagerUtils.isEmptyString(params[4])
					|| p3AppManagerUtils.isEmptyString(params[5])))
			{
				throw new p3AppManagerException("Invalid Parameters. Error: ",
						p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR.getNumVal());
			}
			// set default values for inter run
			p3RunConfig runConfig = new p3RunConfig();
			runConfig.runTime = Double.parseDouble(params[0]);
			runConfig.phaseCorrection = p3Constants.INTER_SPEC_PHASE_CORRECTION;
			runConfig.normalizeSpectrum = p3Constants.INTER_SPEC_NORMALIZE_SPECTRUM;
			runConfig.WL_Correction = p3Constants.INTER_SPEC_WL_CORR;
			runConfig.selfCorrectionEnable = p3Constants.SELF_CORRECTION_DISABLED;
			if(params.length == 6)
			{
				runConfig.apodizationIndex = Integer.parseInt(params[1]);
				runConfig.zeroPaddingMultiplier = Integer.parseInt(params[2]);
				if(Integer.parseInt(params[3]) != -1){
					runConfig.gainIndex = 2;
					runConfig.gainValue = Integer.parseInt(params[3]);
				}else{
					runConfig.gainIndex = 0;
					runConfig.gainValue = 0;
				}
				runConfig.commonWaveNum = Integer.parseInt(params[4]);
				runConfig.mode = Integer.parseInt(params[5]);
			}
			else
			{
				runConfig.apodizationIndex = 0;
				runConfig.zeroPaddingMultiplier = 1;
			}
			runConfig.runType = p3AppManagerState.InterferogramRun;
			
			ApodizationSettings(runConfig);
			SourceSettings(runConfig);
			ScanTimeChecking(runConfig);
			
			return runConfig;

		case RunSpecBackground:

			if (p3AppManagerUtils.isEmptyString(params[0]) || p3AppManagerUtils.isEmptyString(params[1])) {
				throw new p3AppManagerException("Invalid Parameters. Error: ",
						p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR.getNumVal());
			}
			if(params.length == 7 && (p3AppManagerUtils.isEmptyString(params[2])
					|| p3AppManagerUtils.isEmptyString(params[3])
					|| p3AppManagerUtils.isEmptyString(params[4])
					|| p3AppManagerUtils.isEmptyString(params[5])
					|| p3AppManagerUtils.isEmptyString(params[6])))
			{
				throw new p3AppManagerException("Invalid Parameters. Error: ",
						p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR.getNumVal());
			}
			runConfig = new p3RunConfig();
			runConfig.runTime = Double.parseDouble(params[0]);
			runConfig.phaseCorrection = p3Constants.BG_SPEC_PHASE_CORRECTION;
			runConfig.normalizeSpectrum = p3Constants.BG_SPEC_NORMALIZE_SPECTRUM;
			runConfig.WL_Correction = p3Constants.BG_SPEC_WL_CORR;
			runConfig.selfCorrectionEnable = p3Constants.SELF_CORRECTION_DISABLED;
			if(params.length == 7)
			{
				runConfig.apodizationIndex = Integer.parseInt(params[2]);
				runConfig.zeroPaddingMultiplier = Integer.parseInt(params[3]);
				if(Integer.parseInt(params[4]) != -1){
					runConfig.gainIndex = 2;
					runConfig.gainValue = Integer.parseInt(params[4]);
				}else{
					runConfig.gainIndex = 0;
					runConfig.gainValue = 0;
				}
				runConfig.commonWaveNum = Integer.parseInt(params[5]);
				runConfig.mode = Integer.parseInt(params[6]);
			}
			else
			{
				runConfig.apodizationIndex = 0;
				runConfig.zeroPaddingMultiplier = 1;
			}
			runConfig.runType = p3AppManagerState.SpectroscopyBackgroundRun;

			this.setState(p3AppManagerState.SpectroscopyBackgroundRun);

			ApodizationSettings(runConfig);
			SourceSettings(runConfig);
			ScanTimeChecking(runConfig);
			
			return runConfig;

		case RunSpecSample:
			if (p3AppManagerUtils.isEmptyString(params[0]) || p3AppManagerUtils.isEmptyString(params[1])) {
				throw new p3AppManagerException("Invalid Parameters. Error: ",
						p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR.getNumVal());
			}
			if(params.length == 7 && (p3AppManagerUtils.isEmptyString(params[2])
					|| p3AppManagerUtils.isEmptyString(params[3])
					|| p3AppManagerUtils.isEmptyString(params[4])
					|| p3AppManagerUtils.isEmptyString(params[5])
					|| p3AppManagerUtils.isEmptyString(params[6])))
			{
				throw new p3AppManagerException("Invalid Parameters. Error: ",
						p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR.getNumVal());
			}
			runConfig = new p3RunConfig();
			runConfig.runTime = Double.parseDouble(params[0]);
			runConfig.phaseCorrection = p3Constants.INTER_SPEC_PHASE_CORRECTION;
			runConfig.normalizeSpectrum = p3Constants.INTER_SPEC_NORMALIZE_SPECTRUM;
			runConfig.WL_Correction = p3Constants.SPEC_WL_CORR;
			runConfig.selfCorrectionEnable = p3Constants.SELF_CORRECTION_DISABLED;
			if(params.length == 7)
			{
				runConfig.apodizationIndex = Integer.parseInt(params[2]);
				runConfig.zeroPaddingMultiplier = Integer.parseInt(params[3]);
				if(Integer.parseInt(params[4]) != -1){
					runConfig.gainIndex = 2;
					runConfig.gainValue = Integer.parseInt(params[4]);
				}else{
					runConfig.gainIndex = 0;
					runConfig.gainValue = 0;
				}
				runConfig.commonWaveNum = Integer.parseInt(params[5]);
				runConfig.mode = Integer.parseInt(params[6]);
			}
			else
			{
				runConfig.apodizationIndex = 0;
				runConfig.zeroPaddingMultiplier = 1;
			}
			runConfig.runType = p3AppManagerState.SpectroscopySampleRun;

			this.setState(p3AppManagerState.SpectroscopySampleRun);

			ApodizationSettings(runConfig);
			SourceSettings(runConfig);
			ScanTimeChecking(runConfig);
			
			return runConfig;

		case RunWavelengthCalibrationBG:

			if (p3AppManagerUtils.isEmptyString(params[0]) || p3AppManagerUtils.isEmptyString(params[1])
					|| p3AppManagerUtils.isEmptyString(params[2])|| p3AppManagerUtils.isEmptyString(params[3])
					|| p3AppManagerUtils.isEmptyString(params[4])) {
				throw new p3AppManagerException("Invalid Parameters. Error: ",
						p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR.getNumVal());
			}
			runConfig = new p3RunConfig();
			runConfig.runTime = Double.parseDouble(params[0]);
			runConfig.phaseCorrection = p3Constants.BG_SPEC_PHASE_CORRECTION;
			runConfig.normalizeSpectrum = p3Constants.BG_SPEC_NORMALIZE_SPECTRUM;
			runConfig.WL_Correction = p3Constants.BG_SPEC_WL_CORR;
			runConfig.selfCorrectionEnable = p3Constants.SELF_CORRECTION_DISABLED;
			runConfig.apodizationIndex = Integer.parseInt(params[1]);
			runConfig.zeroPaddingMultiplier = Integer.parseInt(params[2]);
			runConfig.runType = p3AppManagerState.wavelengthCalibrationBG_Run;
			if(Integer.parseInt(params[3]) != -1){
				runConfig.gainIndex = 2;
				runConfig.gainValue = Integer.parseInt(params[3]);
			}else{
				runConfig.gainIndex = 0;
				runConfig.gainValue = 0;
			}
			runConfig.commonWaveNum = Integer.parseInt(params[4]);

			this.setState(p3AppManagerState.wavelengthCalibrationBG_Run);

			ApodizationSettings(runConfig);
			SourceSettings(runConfig);
			ScanTimeChecking(runConfig);
			
			return runConfig;

		case RunWavelengthCalibration:
			if (p3AppManagerUtils.isEmptyString(params[0]) || p3AppManagerUtils.isEmptyString(params[1])
					|| p3AppManagerUtils.isEmptyString(params[2]) || p3AppManagerUtils.isEmptyString(params[3])
					|| p3AppManagerUtils.isEmptyString(params[4])|| p3AppManagerUtils.isEmptyString(params[5])) {
				throw new p3AppManagerException("Invalid Parameters. Error: ",
						p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR.getNumVal());
			}
			runConfig = new p3RunConfig();
			runConfig.runTime = Double.parseDouble(params[0]);
			runConfig.standardCalibratorType = params[1];
			runConfig.phaseCorrection = p3Constants.INTER_SPEC_PHASE_CORRECTION;
			runConfig.normalizeSpectrum = p3Constants.INTER_SPEC_NORMALIZE_SPECTRUM;
			runConfig.WL_Correction = p3Constants.SPEC_WL_CORR;
			runConfig.selfCorrectionEnable = p3Constants.SELF_CORRECTION_DISABLED;
			runConfig.apodizationIndex = Integer.parseInt(params[2]);
			runConfig.zeroPaddingMultiplier = Integer.parseInt(params[3]);
			runConfig.runType = p3AppManagerState.wavelengthCalibration_Run;
			if(Integer.parseInt(params[4]) != -1){
				runConfig.gainIndex = 2;
				runConfig.gainValue = Integer.parseInt(params[4]);
			}else{
				runConfig.gainIndex = 0;
				runConfig.gainValue = 0;
			}
			runConfig.commonWaveNum = Integer.parseInt(params[5]);

			this.setState(p3AppManagerState.wavelengthCalibration_Run);

			ApodizationSettings(runConfig);
			SourceSettings(runConfig);
			ScanTimeChecking(runConfig);
			
			return runConfig;

		case RunGainAdjustSpecBG:
			if (p3AppManagerUtils.isEmptyString(params[0])) {
				throw new p3AppManagerException("Invalid Parameters. Error: ",
						p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR.getNumVal());
			}
			// set default values for adaptive gain
			runConfig = new p3RunConfig();
			runConfig.runTime = Double.parseDouble(params[0]);
			runConfig.phaseCorrection = p3Constants.INTER_SPEC_PHASE_CORRECTION;
			runConfig.normalizeSpectrum = p3Constants.INTER_SPEC_NORMALIZE_SPECTRUM;
			runConfig.WL_Correction = p3Constants.INTER_SPEC_WL_CORR;
			runConfig.selfCorrectionEnable = p3Constants.SELF_CORRECTION_DISABLED;
			runConfig.runType = p3AppManagerState.gainAdjustSpecBG_Run;

			ApodizationSettings(runConfig);
			SourceSettings(runConfig);
			ScanTimeChecking(runConfig);
			
			return runConfig;

		case RunSelfCorr:
			if (p3AppManagerUtils.isEmptyString(params[0]) || p3AppManagerUtils.isEmptyString(params[1])
					|| p3AppManagerUtils.isEmptyString(params[2])|| p3AppManagerUtils.isEmptyString(params[3])
					|| p3AppManagerUtils.isEmptyString(params[4])) {
				throw new p3AppManagerException("Invalid Parameters. Error: ",
						p3AppManagerStatus.INVALID_RUN_PARAMETERS_ERROR.getNumVal());
			}
			// set default values for inter run
			runConfig = new p3RunConfig();
			runConfig.runTime = Double.parseDouble(params[0]);
			runConfig.phaseCorrection = p3Constants.INTER_SPEC_PHASE_CORRECTION;
			runConfig.normalizeSpectrum = p3Constants.INTER_SPEC_NORMALIZE_SPECTRUM;
			runConfig.WL_Correction = p3Constants.INTER_SPEC_WL_CORR;
			runConfig.selfCorrectionEnable = p3Constants.SELF_CORRECTION_ENABLED;
			runConfig.apodizationIndex = Integer.parseInt(params[1]);
			runConfig.zeroPaddingMultiplier = Integer.parseInt(params[2]);
			runConfig.runType = p3AppManagerState.selfCorr_Run;
			if(Integer.parseInt(params[3]) != -1){
				runConfig.gainIndex = 2;
				runConfig.gainValue = Integer.parseInt(params[3]);
			}else{
				runConfig.gainIndex = 0;
				runConfig.gainValue = 0;
			}
			runConfig.commonWaveNum = Integer.parseInt(params[4]);

			ApodizationSettings(runConfig);
			SourceSettings(runConfig);
			ScanTimeChecking(runConfig);
			
			return runConfig;

		default:
			return null;

		}
	}

	void ScanTimeChecking(p3RunConfig runConfig) throws p3AppManagerException
	{
		if(runConfig.runTime < p3Constants.MIN_RUNTIMR_MS || runConfig.runTime > p3Constants.MAX_RUNTIME_MS)
		{
			throw new p3AppManagerException("Invalid Scan Time. Error: ",
					p3AppManagerStatus.INVALID_RUN_TIME_ERROR.getNumVal());
		}
		
	}
	
	void ApodizationSettings(p3RunConfig runConfig) throws p3AppManagerException
	{
		if(runConfig.apodizationIndex == 4)
		{
			runConfig.externalApodizationWindow = externalApodizationWindow;
		}
		else
		{
			externalApodizationWindow = null;
		}
		if(runConfig.apodizationIndex == 4 && runConfig.externalApodizationWindow == null)
		{
			throw new p3AppManagerException("Invalid Apodization Window. Error: ",
					p3AppManagerStatus.INVALID_APODIZATION_WINDOW.getNumVal());
		}
		
	}
	
	void SourceSettings(p3RunConfig runConfig)
	{
		double[] delays = p3AppManagerUtils.loadParamFile(p3Constants.SOURCE_DELAY_FILE_PATH);
		
		runConfig.t1 = (int)delays[0];
		runConfig.t2_c1 = (int)delays[1];
		runConfig.t2_c2 = (int)delays[2];
		runConfig.t2_tmax = (int)delays[3];
		runConfig.delta_t = (int)delays[4];
		runConfig.lamps_count = (int)delays[5];
		runConfig.lamp_sel = (int)delays[6];
	}
	/*
	 * ! This function is responsible for set final result depend on type and
	 * notify all observers
	 * 
	 * @param [IN] action Type of the action you performed
	 * 
	 * @param [IN] res data result of the action you performed
	 * 
	 * @param [IN] status is the status of the performed action
	 */
	private void setFinalResultAndnotifyObservers(p3DeviceAction action, double[][] res, p3AppManagerStatus status) {
		setFinalResultByRunType(action, res);
		// set state back to idle
		setState(p3AppManagerState.Idle);
		p3DeviceNotificationResult notification = createSampleNotificationResult(action, status);
		setChanged();
		notifyObservers(notification);
	}

	/*
	 * ! This function is responsible for set final result depend on type
	 * 
	 * @param [IN] action Type of the action you performed
	 * 
	 * @param [IN] result data result of the action you performed
	 */
	private void setFinalResultByRunType(p3DeviceAction action, double[][] result) {
		switch (action) {
		case initializeCore:
			break;
		case RunInterSpec:
			interSpecResult = result;
			break;
		case RunUpdateFFT_SettingsInterSpec:
			interSpecResult = result;
			break;
		case RunUpdateFFT_SettingsSpec:
			SpecResult = result;
			break;
		case RunSpecBackground:
			bgSpecResult = result;
			break;
		case RunSpecSample:
			SpecResult = result;
			break;
		case RunWavelengthCalibrationBG:
			bgWL_CalibResult = result;
			break;
		case RunWavelengthCalibration:
			// Do nothing
			break;
		case RunGainAdjustSpecBG:
		case RunGainAdjustSpecSample:
			GainAdjustSpec_Result = result;
			break;
		case RunSelfCorr:
			interSpecResult = result;
			break;
		default:
			// not supported action
			;
		}
	}

	
	/*
	 * ! This function is responsible for create notification object
	 * 
	 * @param [IN] action Type of the action you performed
	 * 
	 * @param [IN] status is the status of the performed action
	 * 
	 * @return notification object which will be used to notify all your
	 * observers
	 */
	private p3DeviceNotificationResult createSampleNotificationResult(p3DeviceAction action,
			p3AppManagerStatus status) {

		p3DeviceNotificationResult notificationResult = new p3DeviceNotificationResult();
		notificationResult.setAction(action);
		notificationResult.setDeviceId(sampleId);
		notificationResult.setStatus(status);
		return notificationResult;
	}


}
