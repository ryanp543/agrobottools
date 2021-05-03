package sws.p3AppManager_micro.devices;

import java.util.Observable;

import sws.p3AppManager_micro.utils.p3Enumerations.p3AppManagerStatus;

public abstract class p3Device extends Observable {
	// / <summary>
	// / This function is responsible for return device id
	// / </summary>
	// / <returns>device Id </returns>
	public abstract String getDeviceId();

	// / <summary>
	// / This function is responsible for board initialization
	// / </summary>
	// / <returns>application status </returns>
	public abstract p3AppManagerStatus initializeCore(String... params);

	// / <summary>
	// / This function is responsible for checking device status
	// / </summary>
	// / <returns>application status </returns>
	public abstract p3AppManagerStatus checkDeviceStatus(String... params);

	// / <summary>
	// / This function is responsible for run Spec analysis
	// / </summary>
	// / <param name="params"> user parameters input</param>
	// / <returns>application status</returns>
	public abstract p3AppManagerStatus runSpec(String... params);
	
	// / <summary>
	// / This function is responsible for injecting apodization Window
	// / </summary>
	// / <param name="params"> user parameters input</param>
	// / <returns>application status</returns>
	public abstract void setExternalApodizationWindow(long[] apodizationWindow);
	
	// / <summary>
	// / This function is responsible for background measurement for wavelength calibration in spectrum tab
	// / </summary>
	// / <param name="params"> user parameters input</param>
	// / <returns>application status</returns>
	public abstract p3AppManagerStatus wavelengthCalibrationBG(String... params);
	
	// / <summary>
	// / This function is responsible for wavelength calibration in spectrum tab
	// / </summary>
	// / <param name="params"> user parameters input</param>
	// / <returns>application status</returns>
	public abstract p3AppManagerStatus wavelengthCalibration(String... params);
	
	// / <summary>
	// / This function is responsible for get transmitance error
	// / </summary>
	// / <returns>error</returns>
	public abstract double getErrorData();

	// / <summary>
	// / This function is responsible for get spec result
	// / </summary>
	// / <param name="res">object to set data </param>
	// / <returns>application status</returns>
	public abstract double[][] getSpecData();

	// / <summary>
	// / This function is responsible for run inter Spec analysis
	// / </summary>
	// / <param name="params"> user parameters input</param>
	// / <returns>application status</returns>
	public abstract p3AppManagerStatus runInterSpec(String... params);
	
	// / <summary>
	// / This function is responsible for run Interferogram correction calculation
	// / </summary>
	// / <param name="params"> user parameters input</param>
	// / <returns>application status</returns>
	public abstract p3AppManagerStatus runCalibCorr(String... params);
	
	// / <summary>
	// / This function is responsible for run BG gain adjustment in Spec panel
	// / </summary>
	// / <param name="params"> user parameters input</param>
	// / <returns>application status</returns>
	public abstract p3AppManagerStatus runSpecGainAdjBG(String... params);
	
	// / <summary>
	// / This function is responsible for get raw data
	// / </summary>
	// / <returns>raw data</returns>
	public abstract double[][] getRawData();
	
	// / <summary>
	// / This function is responsible for get inter spec result
	// / </summary>
	// / <param name="res">object to set data </param>
	// / <returns>application status</returns>
	public abstract double[][] getInterSpecData();
		
	// / <summary>
	// / This function is responsible for getting gain adjustment data for Spec Panel
	// / </summary>
	// / <param name="res">object to set data </param>
	// / <returns>application status</returns>
	public abstract double[][] getGainAdjustSpecData();

	// / <summary>
	// / This function is responsible for restoring default settings
	// / </summary>
	// / <param name="params"> user parameters input</param>
	// / <returns>application status</returns>
	public abstract p3AppManagerStatus restoreDefaultSettings(String... params);	

	// / <summary>
	// / This function is responsible for burn working settings
	// / </summary>
	// / <param name="params"> user parameters input</param>
	// / <returns>application status</returns>	
	public abstract p3AppManagerStatus burnSettingsSpecificSettings(String... params);
	
	// / <summary>
	// / This function is responsible for getting pi and EVK versions
	// / </summary>
	// / <returns>String array</returns>
	public abstract String[] getSoftwareVersion();
	
	// / <summary>
	// / This function is responsible for sleep action
	// / </summary>
	// / <returns>application status</returns>
	public abstract p3AppManagerStatus sleepAction();
	
	// / <summary>
	// / This function is responsible for wake up action
	// / </summary>
	// / <returns>application status</returns>
	public abstract p3AppManagerStatus wakeUpAction();
	
	// / <summary>
	// / This function is responsible for power off
	// / </summary>
	// / <returns>application status</returns>
	public abstract p3AppManagerStatus powerOff();
	
	// / <summary>
	// / This function is responsible for power on
	// / </summary>
	// / <returns>application status</returns>
	public abstract p3AppManagerStatus powerOn();
	
}

