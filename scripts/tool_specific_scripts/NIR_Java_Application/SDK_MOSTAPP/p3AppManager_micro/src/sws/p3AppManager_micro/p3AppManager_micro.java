package sws.p3AppManager_micro;

import java.util.Observable;
import java.util.Observer;

import sws.p3AppManager_micro.utils.p3Enumerations.p3AppManagerStatus;


public abstract class p3AppManager_micro  extends Observable implements  Observer{
	
	/*
	 * ! This function is responsible for setting the working directory
	 * 
	 */
	public abstract void setWorkingDirectory(String dir);
	
	/*
	 * ! This function is responsible for getting the working directory
	 * @return working directory
	 */
	public abstract String getWorkingDirectory(String dir);
	
	/*
	 * ! This function is responsible for return the device id
	 * @return device id 
	 */
	public abstract String getDeviceId();

	/*
	 * ! This function is responsible for board initialization
	 * @return Result of the operation
	 */
	public abstract p3AppManagerStatus initializeCore(String ... param);

	/*
	 * ! This function is responsible for checking device status
	 * @return Result of the operation
	 */
	public abstract p3AppManagerStatus checkDeviceStatus(String ... param);
	
	/*
	 * ! This function is responsible for injecting apodization Window
	 * @return Result of the operation
	 */
	public abstract void setExternalApodizationWindow(long[] apodizationWindow);

	/*
	 * ! This function is responsible for run Spec analysis
	 * @return Result of the operation
	 */
	public abstract p3AppManagerStatus runSpec(String ... params);
	
	/*
	 * ! This function is responsible for wavelength calibration in spectrum tab
	 * @return Result of the operation
	 */
	public abstract p3AppManagerStatus wavelengthCalibration(String ... params);
	
	/*
	 * ! This function is responsible for background measurement in wavelength calibration in spectrum tab
	 * @return Result of the operation
	 */
	public abstract p3AppManagerStatus wavelengthCalibrationBG(String ... params);

	/*
	 * ! This function is responsible for check if the application will run with background data
	 * @return object represent data 
	 */
	public abstract  double[][] getSpecData();
	
	/*
	 * ! This function is responsible for return transmittance error
	 * @return object represent data 
	 */
	public abstract  double getErrorData();
	
	/*
	 * ! This function is responsible for run inter Spec analysis
	 * @return Result of the operation
	 */
	public abstract p3AppManagerStatus runInterSpec(String ... params);
	
	/*
	 * ! This function is responsible for run Interferogram correction calculation
	 * @return Result of the operation
	 */
	public abstract p3AppManagerStatus runCalibCorr(String ... params);
	
	/*
	 * ! This function is responsible for BG gain adjustment in Spec panel
	 * @return Result of the operation
	 */
	public abstract p3AppManagerStatus runSpecGainAdjBG(String ... params);
	
	/*
	 * ! This function is responsible for get inter spec result
	 * @return object represent data 
	 */
	public abstract double[][] getInterSpecData();
	
	/*
	 * ! This function is responsible for get gain adjustment result for Spec panel
	 * @return object represent data 
	 */
	public abstract double[][] getGainAdjustSpecData();

	/*
	 * ! This function is responsible for restoring default settings
	 * 
	 * @param result Result of the operation
	 * 
	 * @return This function returns nothing
	 */
	public abstract p3AppManagerStatus restoreDefaultSettings(String... params);

	/*
	 * ! This function is responsible for burning working settings
	 * 
	 * @param result Result of the operation
	 * 
	 * @return This function returns nothing
	 */
	public abstract p3AppManagerStatus burnSpecificSettings(String... params);
	
	/*
	 * ! This function is responsible for getting pi and EVK versions
	 * @return String array
	 */
	public abstract String[] getSoftwareVersion();
	
	/*
	 * ! This function is responsible for sleep action
	 * @return Result of the operation
	 */
	public abstract p3AppManagerStatus sleepAction();
	
	/*
	 * ! This function is responsible for wake up action
	 * @return Result of the operation
	 */
	public abstract p3AppManagerStatus wakeUpAction();
	
	/*
	 * ! This function is responsible for power off
	 * @return Result of the operation
	 */
	public abstract p3AppManagerStatus powerOff();
	
	/*
	 * ! This function is responsible for power on
	 * @return Result of the operation
	 */
	public abstract p3AppManagerStatus powerOn();

}
