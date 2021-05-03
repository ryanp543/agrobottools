package sws.p3AppManager_micro;

import java.util.Observable;

import org.apache.log4j.Logger;

import sws.p3AppManager_micro.devices.p3Device;
import sws.p3AppManager_micro.devices.p3Sample;
import sws.p3AppManager_micro.utils.p3AppManagerNotification;
import sws.p3AppManager_micro.utils.p3AppManagerUtils;
import sws.p3AppManager_micro.utils.p3Constants;
import sws.p3AppManager_micro.utils.p3DeviceNotificationResult;
import sws.p3AppManager_micro.utils.p3Enumerations.p3AppManagerStatus;
import sws.p3AppManager_micro.utils.p3Enumerations.p3DeviceAction;

public class p3AppManagerImpl extends p3AppManager_micro {



	// INFO : could be list to manage more than one device
	private p3Device device;

	private static Logger logger = Logger.getLogger(p3AppManagerImpl.class);

	private String workingDirectory;
	/*
	 * ! This function is responsible for p3AppManager construction
	 * @return This function returns nothing
	 */
	public p3AppManagerImpl() {

		// initialize general configuration and dependencies
		workingDirectory = System.getProperty("user.dir");
		p3Constants.APPLICATION_WORKING_DIRECTORY = workingDirectory;
	}

	public p3AppManagerImpl(String dir) {
		
		// initialize general configuration and dependencies
		workingDirectory = dir;
		p3Constants.APPLICATION_WORKING_DIRECTORY = dir;
	}

	
	/*
	 * ! This function is responsible for board initialization
	 * @return p3AppManagerStatus
	 */
	@Override
	public p3AppManagerStatus initializeCore(String... params) { // optional
		// params

		logger.info("start initializeCore method ");
		// check and get the connected device
		if (this.device == null) {

			this.device = getConnectedSample();

		}

		p3AppManagerStatus result = p3AppManagerStatus.NO_ERROR;
		
		return result;

	}

	/*
	 * ! This function is responsible for checking the status of the connected device
	 * @return p3AppManagerStatus
	 */
	@Override
	public p3AppManagerStatus checkDeviceStatus(String... params) {// optional
		// params

		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.checkDeviceStatus(params);

	}
	/*
	 * ! This function is responsible for injecting apodization Window
	 * @return void
	 */
	@Override
	public void setExternalApodizationWindow(long[] apodizationWindow)
	{
		logger.info("start setExternalApodizationWindow method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		this.device.setExternalApodizationWindow(apodizationWindow);
	}
	
	/*
	 * ! This function is responsible for run Spec analysis
	 * @return p3AppManagerStatus
	 */
	@Override
	public p3AppManagerStatus runSpec(String... params) {// 2 mandatory params
		// (time, isSample)

		logger.info("start runSpec method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.runSpec(params);

	}
	
	/*
	 * ! This function is responsible for background measurement for wavelength calibration in spectrum tab
	 * @return p3AppManagerStatus
	 */
	@Override
	public p3AppManagerStatus wavelengthCalibrationBG(String... params) {// 3 mandatory params
		// (time, standardCalibrator)
		
		logger.info("start wavelengthCalibrationBG method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.wavelengthCalibrationBG(params);
		
	}
	
	/*
	 * ! This function is responsible for wavelength calibration in spectrum tab
	 * @return p3AppManagerStatus
	 */
	@Override
	public p3AppManagerStatus wavelengthCalibration(String... params) {// 4 mandatory params
		// (time, standardCalibrator)
		
		logger.info("start wavelengthCalibration method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.wavelengthCalibration(params);
		
	}
	/*
	 * ! This function is responsible for get transmitance error
	 * @return p3AppManagerStatus
	 */
	@Override
	public double getErrorData() {
		logger.info("start getErrorData method ");
		if (this.device == null) {
			return 1;
		}
		return this.device.getErrorData();
	}

	/*
	 * ! This function is responsible for get spec result
	 * @return p3AppManagerStatus
	 */
	@Override
	public double[][] getSpecData() {
		logger.info("start getSpecData method ");
		if (this.device == null) {
			return null;
		}
		return this.device.getSpecData();
	}
	
	/*
	 * ! This function is responsible for run inter Spec analysis
	 * @return p3AppManagerStatus
	 */
	@Override
	public p3AppManagerStatus runInterSpec(String... params) {// 3 mandatory param
		// (time)

		logger.info("start runInterSpec method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.runInterSpec(params);
	}
	
	/*
	 * ! This function is responsible for run Interferogram correction calculation
	 * @return p3AppManagerStatus
	 */
	@Override
	public p3AppManagerStatus runCalibCorr(String... params) {// 1 mandatory param
		// (time)

		logger.info("start runCalibCorr method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.runCalibCorr(params);
	}
	
	/*
	 * ! This function is responsible for run BG gain adjustment in Spec panel
	 * @return p3AppManagerStatus
	 */
	@Override
	public p3AppManagerStatus runSpecGainAdjBG(String... params) {// 1 mandatory param
		// (time)
		
		logger.info("start runSpecGainAdjBG method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		
		return this.device.runSpecGainAdjBG(params);
	}
	
	/*
	 * ! This function is responsible for get inter spec result
	 * @return interSpec data
	 */
	@Override
	public double[][] getInterSpecData() {
		logger.info("start getInterSpecData method ");
		if (this.device == null) {
			return null;
		}
		return this.device.getInterSpecData();
	}
	
	/*
	 * ! This function is responsible for getting gain adjustment data for Spec Panel
	 * @return gain adjustment data
	 */	
	@Override
	public double[][] getGainAdjustSpecData() {
		logger.info("start getGainAdjustSpecData method ");
		if (this.device == null) {
			return null;
		}
		return this.device.getGainAdjustSpecData();
	}

	/*
	 * ! This function is responsible for getting module ID
	 * @return module ID
	 */	
	@Override
	public String getDeviceId()
	{
		return this.device.getDeviceId();
	}

	/*
	 * ! This function is responsible for updating the SW based on the needed action
	 * @return module ID
	 */	
	@Override
	public void update(Observable arg0, Object result) {

		logger.info("start update method ");
		if (result == null || !(result instanceof p3DeviceNotificationResult)) {

			this.setChanged();
			this.notifyObservers(new p3AppManagerNotification(-1,
					p3AppManagerStatus.INVALID_NOTIFICATION_ERROR.getNumVal(), null));
		}
		p3DeviceNotificationResult notificationResult = (p3DeviceNotificationResult) result;

		p3DeviceAction action = notificationResult.getAction();
		if (action == null) {
			this.setChanged();
			this.notifyObservers(new p3AppManagerNotification(-1, p3AppManagerStatus.INVALID_ACTION_ERROR.getNumVal(), null));
		}

		String deviceId = notificationResult.getDeviceId();
		if (p3AppManagerUtils.isEmptyString(deviceId)) {
			this.setChanged();
			this.notifyObservers(new p3AppManagerNotification(-1, p3AppManagerStatus.INVALID_DEVICE_ERROR.getNumVal(), null));
		}

		switch (action) {
		
		case initializeCore:
			// call method to perform initial status
			logger.info("initialize core....");
			initializeCoreFinished(deviceId, notificationResult.getStatus());
			break;
		case RunInterSpec:

			logger.info("run inter spec....");
			RunInterSpecFinished(deviceId, notificationResult.getStatus());
			break;
		
		case RunGainAdjustSpecBG:
			
			logger.info("run Gain Adjustment Spec BG....");
			RunGainAdjustSpecBG_Finished(deviceId, notificationResult.getStatus());
			break;
			
		case RunSpecBackground:

			logger.info("run background ....");
			RunSpecBackgroundeFinished(deviceId, notificationResult.getStatus());
			break;
		case RunSpecSample:

			logger.info("run spectrocopy ....");
			RunSpecSampleFinished(deviceId, notificationResult.getStatus());
			break;
		case RunWavelengthCalibrationBG:
			
			logger.info("run BG wavelength calibration ....");
			RunWavelengthCalibrationBG_Finished(deviceId, notificationResult.getStatus());
			break;
		case RunWavelengthCalibration:
			
			logger.info("run wavelength calibration ....");
			RunWavelengthCalibrationFinished(deviceId, notificationResult.getStatus());
			break;
		
		case RestoreDefaultSettings:
			logger.info("Restore Default Settings");
			RestoreDefaultSettingsFinished(deviceId, notificationResult.getStatus());
			break;
		case RunSelfCorr:
			
			logger.info("run Interferogram Corr ....");
			RunInterferogramCorr_Finished(deviceId, notificationResult.getStatus());
			break;
			
		case BurnWorkingSettings:	
			logger.info("run BurnWorkingSettings ....");
			BurnWorkingSettings_Finished(deviceId, notificationResult.getStatus());
			break;
		case SleepAction:
			logger.info("run sleepAction ....");
			RunSleepAction_Finished(deviceId, notificationResult.getStatus());
			break;
		case WakeUpAction:
			logger.info("run wakeUpAction ....");
			RunWakeUpAction_Finished(deviceId, notificationResult.getStatus());
			break;
		case PowerOff:
			logger.info("run powerOff ....");
			PowerOff_Finished(deviceId, notificationResult.getStatus());
			break;
		case PowerOn:
			logger.info("run powerOn ....");
			PowerOn_Finished(deviceId, notificationResult.getStatus());
			break;
		default:
			this.setChanged();
			this.notifyObservers(new p3AppManagerNotification(-1, p3AppManagerStatus.INVALID_ACTION_ERROR.getNumVal(), null));
		}

	}

	
	/*
	 * ! This function is responsible for checking connected device and create
	 * new sample for it
	 * @return device object by sample id
	 */
	private p3Device getConnectedSample() {

		p3Device device = new p3Sample();
		device.addObserver(this);
		return device;
	}

	/*
	 * ! This function is responsible for restoring default settings
	 * @return p3AppManagerStatus
	 */	
	@Override
	public p3AppManagerStatus restoreDefaultSettings(String... params)
	{
		logger.info("start restoreDefaultSettings method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.restoreDefaultSettings(params);
	}

	/*
	 * ! This function is responsible for sleep action
	 * @return p3AppManagerStatus
	 */	
	@Override
	public p3AppManagerStatus sleepAction()
	{
		logger.info("start sleepAction method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.sleepAction();
	}
	
	/*
	 * ! This function is responsible for wake up action
	 * @return p3AppManagerStatus
	 */	
	@Override
	public p3AppManagerStatus wakeUpAction()
	{
		logger.info("start wakeUpAction method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.wakeUpAction();
	}
	
	/*
	 * ! This function is responsible for power off
	 * @return p3AppManagerStatus
	 */	
	@Override
	public p3AppManagerStatus powerOff()
	{
		logger.info("start powerOff method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.powerOff();
	}
	
	/*
	 * ! This function is responsible for power on
	 * @return p3AppManagerStatus
	 */	
	@Override
	public p3AppManagerStatus powerOn()
	{
		logger.info("start powerOn method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.powerOn();
	}
	
	/*
	 * ! This function is responsible for burn working settings
	 * @return p3AppManagerStatus
	 */	
	@Override
	public p3AppManagerStatus burnSpecificSettings(String... params)
	{
		logger.info("start restoreDefaultSettings method ");
		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.burnSettingsSpecificSettings(params);
	}
	
	/*
	 * ! This function is responsible for setting working directory
	 * @return void
	 */	
	@Override
	public void setWorkingDirectory(String dir) {
		this.workingDirectory = dir;
		p3Constants.APPLICATION_WORKING_DIRECTORY = dir;
	}

	/*
	 * ! This function is responsible for getting working directory
	 * @return String
	 */	
	@Override
	public String getWorkingDirectory(String dir) {
		
		return this.workingDirectory;
	}
	
	/*
	 * ! This function is responsible for getting pi and EVK versions
	 * @return String Array
	 */	
	@Override
	public String[] getSoftwareVersion() {

		if (this.device == null) {
			this.device = getConnectedSample();
		}
		return this.device.getSoftwareVersion();

	}
	
	/************************************************* status methods **************************************************/
	// INFO : In order to change the object that we use to notify our observers
	// (p2AppManagerNotification)
	// we can arrange every parameter in that class in a string array.
	private void initializeCoreFinished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("lynxinitializeCoreFinished");
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.initializeCore.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}

	private void RunInterSpecFinished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("RunInterSpecFinished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.RunInterSpec.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}
	
	private void RunGainAdjustSpecBG_Finished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("RunGainAdjustSpecBG_Finished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.RunGainAdjustSpecBG.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}

	private void RunSpecBackgroundeFinished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("RunSpecBackgroundeFinished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.RunSpecBackground.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}
	
	private void RunSpecSampleFinished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("RunSpecSampleFinished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.RunSpecSample.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}
	
	private void RunWavelengthCalibrationBG_Finished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("RunWavelengthCalibrationBG_Finished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.RunWavelengthCalibrationBG.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}
	
	private void RunWavelengthCalibrationFinished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("RunWavelengthCalibrationFinished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.RunWavelengthCalibration.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}

	private void RestoreDefaultSettingsFinished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("RestoreDefaultSettingsFinished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.RestoreDefaultSettings.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}
	
	private void RunInterferogramCorr_Finished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("RunInterferogramCorr_Finished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.RunSelfCorr.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}
	
	private void BurnWorkingSettings_Finished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("RunInterferogramCorr_Finished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.BurnWorkingSettings.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}
	
	private void RunSleepAction_Finished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("RunSleepAction_Finished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.SleepAction.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}
	
	private void RunWakeUpAction_Finished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("RunWakeUpAction_Finished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.WakeUpAction.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}
	
	private void PowerOff_Finished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("PowerOff_Finished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.PowerOff.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}
	
	private void PowerOn_Finished(String deviceId, p3AppManagerStatus appManagerStatus) {
		logger.info("PowerOn_Finished with status : " + appManagerStatus);
		this.setChanged();
		this.notifyObservers(new p3AppManagerNotification(p3DeviceAction.PowerOn.getNumVal(), appManagerStatus
				.getNumVal(), deviceId));
	}
	
}
