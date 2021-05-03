package sws.p3AppManager_micro.utils;

import java.io.File;

import sws.p3AppManager_micro.utils.p3Enumerations.p3CorrectionType;

public class p3Constants {

	/********************* needed files paths templates *********************/
	public static String APPLICATION_WORKING_DIRECTORY = System.getProperty("user.dir");
	public static final String Original_APPLICATION_WORKING_DIRECTORY = System.getProperty("user.dir");

	public static final String	PARAM_HEADER_FIE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + p3Constants.MEMS_FLODER + File.separatorChar
			+ p3Constants.PARAM_COFIG_FILE;

	public static final String CONFIG_SAMPLE_PATH_TEMPLATE = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + p3Constants.CONF_FILES_FLODER + File.separatorChar + "{0}"; // sampleID


	public static final String	CONFIG_SAMPLE_PROFILE_PATH_TEMPLATE	= Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + p3Constants.CONF_FILES_FLODER + File.separatorChar + "{0}"
			+ File.separatorChar + "{1}"; // sampleId, profileName (temp + resolution)
	public static final String	TEMP_CONFIG_SAMPLE_PATH	= Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + p3Constants.CONF_FILES_FLODER + File.separatorChar + "TempConf"; 
	
	
	public static final String CONFIG_SAMPLE_PROFILE_FILES_PATH_TEMPLATE = Original_APPLICATION_WORKING_DIRECTORY + 
			File.separatorChar + p3Constants.CONF_FILES_FLODER + File.separatorChar + "{0}" // sampleID
			+ File.separatorChar + "{1}"// profile (temp +  resolution)
			+ File.separatorChar + "{2}";// file

	public static final String GLOBAL_CONFIG_FILES_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER  + File.separatorChar + "{0}";

	public static final String	GLOBAL_CONFIG_MEMS_PATH	= Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + p3Constants.MEMS_FLODER;

	public static final String REGISTERS_OPTIONS_FOLDER_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER  + File.separatorChar + p3Constants.REGISTERS;

	public static final String CONF_FILES_FLODER = "Conf_Files";
	public static final String MEMS_FLODER = "mems";
	public static final String WHITE_LIGHT_FILES_DIR = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ "white Light";
	public static final String WHITE_LIGHT_FILE = "WhiteLight.xml";

	public static final String OPTICAL_SETTINGS_FILE_NAME = "savedOpticalSettings.txt";
	public static final String OPTICAL_SETTINGS_TEMP_FILE_NAME = "tempOpticalSettings.txt";
	public static final String OPTICAL_SETTINGS_TEMP_FILE_NAME2 = "tempOpticalSettings2.txt";
	
	public static final String OPTICAL_SETTINGS_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + "Conf_Files" + File.separatorChar + "{0}" + File.separatorChar + OPTICAL_SETTINGS_FILE_NAME;
	public static final String OPTICAL_SETTINGS_TEMP_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + "Conf_Files" + File.separatorChar + "TempConf" + File.separatorChar + OPTICAL_SETTINGS_TEMP_FILE_NAME;
	public static final String OPTICAL_SETTINGS_TEMP2_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + "Conf_Files" + File.separatorChar + "TempConf" + File.separatorChar + OPTICAL_SETTINGS_TEMP_FILE_NAME2;
	
	public static final String OPTICAL_SETTINGS_CURRENT_RANGE = "CURRENT_RANGE";

	public static final String OPTICAL_SETTINGS_PGA1 = "PGA1";
	
	public static final String OPTICAL_SETTINGS_PGA2 = "PGA2";
	
	public static final String STANDARD_CALIBRATORS_FOLDER_NAME = "standard_calibrators";
	

	public static final String SOURCE_DELAY_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER + File.separatorChar + "sourceSettings.txt";


	/*********************  config files  *********************/
	public static final String PARAM_COFIG_FILE = "param.conf";
	public static final String TREG_COFIG_FILE = "t.reg";

	public static final String C2X_COFIG_FILE = "C2x.cal";
	public static final String CORR_COFIG_FILE = "corr.cal";
	public static final String WL_CORR_COFIG_FILE = "wl_corr.cal";
	public static final String	WAVE_LENGTH_COFIG_FILE = "wavelength_corr.cal";
	public static final String	CALIBRATION	= "Calibration";
	public static final String	REGISTERS	= "Registers";

	/*********************  options blocks and files  *********************/
	public static final String DETECTOR_AND_CURRENT_RANGE_OPTION_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER + File.separatorChar + p3Constants.REGISTERS + File.separatorChar + p3Constants.OPTICAL_PATH_SETTINGS_BLOCK
			+ File.separatorChar + p3Constants.DETECTOR_AND_CURRENT_RANGE_OPTION;

	public static final String PGA1_OPTION_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER + File.separatorChar + p3Constants.REGISTERS + File.separatorChar + p3Constants.OPTICAL_PATH_SETTINGS_BLOCK
			+ File.separatorChar + p3Constants.PGA1_OPTION;

	public static final String PGA2_OPTION_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER + File.separatorChar + p3Constants.REGISTERS + File.separatorChar + p3Constants.OPTICAL_PATH_SETTINGS_BLOCK
			+ File.separatorChar + p3Constants.PGA2_OPTION;

	public static final String ACTUATION_DIRECTION_OPTION_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER + File.separatorChar + p3Constants.REGISTERS + File.separatorChar + p3Constants.OPEN_LOOP_SETTINGS_BLOCK
			+ File.separatorChar + p3Constants.ACTUATION_DIRECTION_OPTION;

	public static final String WAVEFORM_OPTION_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER + File.separatorChar + p3Constants.REGISTERS + File.separatorChar + p3Constants.OPEN_LOOP_SETTINGS_BLOCK
			+ File.separatorChar + p3Constants.WAVEFORM_OPTION;

	public static final String EXCITATION_VOLTAGE_OPTION_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER + File.separatorChar + p3Constants.REGISTERS + File.separatorChar + p3Constants.CAP_SENSING_PATH_SETTINGS_BLOCK
			+ File.separatorChar + p3Constants.EXCITATION_VOLTAGE_OPTION;

	public static final String SAMPLING_RATE_OPTION_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER + File.separatorChar + p3Constants.REGISTERS + File.separatorChar + p3Constants.CAP_SENSING_PATH_SETTINGS_BLOCK
			+ File.separatorChar + p3Constants.SAMPLING_RATE_OPTION;

	public static final String C2V_GAIN_OPTION_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER + File.separatorChar + p3Constants.REGISTERS + File.separatorChar + p3Constants.CAP_SENSING_PATH_SETTINGS_BLOCK
			+ File.separatorChar + p3Constants.C2V_GAIN_OPTION;

	public static final String CLOSED_LOOP_GAIN_VS_FREQ_FILE_PATH	= Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER + File.separatorChar + "moduleSettings" + File.separatorChar + "closedLoopSettings" + File.separatorChar + "bpfGainVsFreq.txt";
	
	public static final String STANDARD_CALIBRATORS_FOLDER_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar
			+ p3Constants.MEMS_FLODER + File.separatorChar + p3Constants.STANDARD_CALIBRATORS_FOLDER_NAME;
	
	public static final String STANDARD_CALIBRATORS_FILE_EXT = ".txt";

	/*********************  options blocks and files  *********************/

	public static final String	OPTICAL_PATH_SETTINGS_BLOCK	= "OpticalPathSettings";
	public static final String	DETECTOR_AND_CURRENT_RANGE_OPTION	= "DetectorAndCurrentRange.txt";
	public static final String	PGA1_OPTION	= "PGA1.txt";
	public static final String	PGA2_OPTION	= "PGA2.txt";

	public static final String	OPEN_LOOP_SETTINGS_BLOCK	= "OpenLoopSettings";
	public static final String	WAVEFORM_OPTION	= "Waveform.txt";
	public static final String	ACTUATION_DIRECTION_OPTION	= "ActuationDirection.txt";

	public static final String	CAP_SENSING_PATH_SETTINGS_BLOCK	= "CapSensingPathSettings";
	public static final String	EXCITATION_VOLTAGE_OPTION	= "ExcitationVoltage.txt";
	public static final String	SAMPLING_RATE_OPTION	= "SamplingRate.txt";
	public static final String	C2V_GAIN_OPTION	= "C2VGain.txt";
	
	/*********************  names of calibration data for laser,white light and Methylene Chloride  *********************/
	public static final String	LASER_FILE = "sl";
	public static final String	WL_FILE = "wl";
	public static final String	METH_FILE = "mc";

	public static final String	CALIBRATION_FOLDER_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + p3Constants.CALIBRATION;

	public static final String	LASER_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + p3Constants.CALIBRATION + File.separatorChar
			+ p3Constants.LASER_FILE;

	public static final String	WL_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + p3Constants.CALIBRATION + File.separatorChar
			+ p3Constants.WL_FILE;

	public static final String	METH_FILE_PATH = Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + p3Constants.CALIBRATION + File.separatorChar
			+ p3Constants.METH_FILE;
	
	public static final String TWO_POINTS_CORR_CALIB_FOLDER_NAME = "two_points_corr";
	
	public static final String STANDARD_CALIBRATOR_DEFAULT_CHOISE = "Choose Material ...";

	/********************* Default inter spec run parameter *********************/

	public static final int INTER_SPEC_WL_CORR = p3CorrectionType.Corrected.getNumVal();
	public static final short INTER_SPEC_NORMALIZE_SPECTRUM = 0;
	public static final short INTER_SPEC_PHASE_CORRECTION = 0;
	public static final short SELF_CORRECTION_DISABLED = 0;
	public static final short SELF_CORRECTION_ENABLED = 1;

	public static final String INTER_SPEC_INTERFERO_DATA_FILE = "Interferogram";

	public static final String INTER_SPEC_SPECTRA_DATA_FILE = "Spectra";

	/********************* Default spec run parameter *********************/
	public static final int SPEC_WL_CORR = p3CorrectionType.Corrected.getNumVal();
	public static final short SPEC_NORMALIZE_SPECTRUM = 0;
	public static final short SPEC_PHASE_CORRECTION = 0;

	public static final String SPEC_SPECTRUM_DATA_FILE = "Spectrum";

	/********************* Default back ground spec run parameter *********************/

	public static final int BG_SPEC_WL_CORR = p3CorrectionType.Corrected.getNumVal();
	public static final short BG_SPEC_NORMALIZE_SPECTRUM = 0;
	public static final short BG_SPEC_PHASE_CORRECTION = 0;
	
	/********************* Default generate calib parameter *********************/
	
	public static final int CALIB_WL_CORR = p3CorrectionType.Corrected.getNumVal();

	/********************* TAIFDriver data parameter *********************/
	// Indices of DSP post processing arrays to
	public static final int OPTICAL_PATH_DIFFERENCE_INDEX = 0;
	public static final int I_INDEX = 1;
	public static final int WAVENUMBER_INDEX = 2;
	public static final int POWER_SPECTRAL_DENSITY_INDEX = 3;
	public static final int POWER_SPECTRAL_DENSITY_EXTENDED_INDEX = 5;
	public static final int CORR_FACTORS_INDEX = 4;

	/***************************** DSP p2Constants ****************************/
	//length of DSP arrays to operate on
	public static final int DSP_DATA_LENGTH = 5;

	//length of final DSP arrays of DSP to be sent back to the request owner
	public static final int DSP_FINAL_DATA_LENGTH = 5;

	public static final int DSP_FFT_DATA_LENGTH = 4;

	public static final int DSP_CALIB_CAP_TIME_DATA_LENGTH = 7;
	public static final int DSP_CALIB_DELAY_COMP_DATA_LENGTH = 3;
	public static final int DSP_CALIB_DATA_LENGTH = 7;

	public static final int DSP_CAP_CURRENT_DATA_LENGTH = 12;

	public static final int DSP_WL_CORR_LENGTH = 10;

	public static final int DSP_DISPLAY_DATA_LENGTH = 4;
	
	public static final int DSP_SNR_DATA_LENGTH = 17;

	public static final int DSP_STABILITY_DATA_LENGTH = 8;
	
	/*********** default register file to be sent after initialization *****/
	public static final String DEFAULT_REG_FILE_FOLDER = "TAIFReg";

	public static final int REG_FILE_COUNT = 89;

	/********************* interval time calculation *********************/
	public static double MAX_RUNTIME_MS = 28000;

	public static final int MIN_RUNTIMR_MS = 10;
	
	//swallow 3 times. in the 4th one, throw
	public static final int MAX_ERROR_COUNT = 4;

	// MAX NO OF SLICES * 1000
	public static final int MAX_NO_OF_SLICES_1000 = 16384000; //= 1024 * 16 * 1000
	public static final int MAX_INT_THRESHOLD_CALIBRATION = (int) Math.pow(2, 11);
	
	public static final int DMUX_MODE_CAPLPF = 15;
	public static final int DMUX_MODE_AMP = 23;
	public static final int DMUX_MODE_AACOUT = 27;

	public static final int MAX_NO_OF_PROFILES_ON_ROM = 8;	//Max no of profiles that could be burned on ROM

	//Max LSB window to add a calibration folder to it
	public static final int MAX_LSB_WINDOW_CALIBRATION = 72000;
	/*!
	 * 	 temperature window to eliminate selection of different temperature sample
	 * 	 folder every time if the current temperature is on the exact boundaries
	 */
	public static final int	TEMPERATURE_WINDOW = 20;

	public static final double MAX_TEMP_LSB = 134217727;//2047//268435455

	/********************* ----------------- *********************/
	
	/* To be updated after DSP implementation { "Rectangular", "Tukey .25", "Borton-Beer Medium", "Gaussian", "Happ-Genzel", "Cosine", "Blackman-Harris", "Triangular"}*/
	//public static final String[] apodizationOptions = new String[] { "Boxcar", "Gaussian", "Happ-Genzel", "Lorenz", "External Function"};
	public static final String[] apodizationOptions = new String[] { "Boxcar", "Gaussian", "Happ-Genzel", "Lorenz"};
	public static final int apodizationDefaultIndex = 0;
	public static final int EXTERNAL_WINDOW_MAX_LENGTH = 20;

	public static final String[] paddingOptions = new String[] { "0", "1", "3"};
	public static final int paddingDefaultIndex = 1;

	public static final String[] commonWavenumberOptions = new String[] { "65 pts", "129 pts", "257 pts", "513 pts", "1024 pts", "2048 pts", "4096 pts"};
	
	public static final int SUB_PANELS_DIMENTION = 300;
	
	public static final int MAX_WIDTH_OF_FIELD = 110;

	public static final int PARAM_CONF_INDEX_interpolationThreshold = 1;
	public static final int PARAM_CONF_INDEX_I_adc_gain = 9;
	public static final int PARAM_CONF_INDEX_act_freq = 11;
	public static final int PARAM_CONF_INDEX_time_initialdly = 12;
	public static final int PARAM_CONF_INDEX_xinterp_lim_min = 14;
	public static final int PARAM_CONF_INDEX_xinterp_lim_max = 15;
	public static final int PARAM_CONF_INDEX_NPs_interp = 17;
	public static final int PARAM_CONF_INDEX_closedLoop = 47;
	public static final int PARAM_CONF_INDEX_KiStartUpFactor = 48;
	public static final int PARAM_CONF_INDEX_OpticalGainMargin = 49;
	public static final int PARAM_CONF_INDEX_LambdaMin = 23;
	public static final int PARAM_CONF_INDEX_LambdaMax = 24;
	public static final int PARAM_CONF_INDEX_CUSTOMER_CORR_SELECT = 56;
	//public static final int PARAM_CONF_INDEX_IN_SAMPLE = 25;
	public static final int PARAM_CONF_INDEX_Apodization = 19;
	public static final int PARAM_CONF_INDEX_ZeroPadding = 22;
	public static final int PARAM_CONF_INDEX_saturationThr = 44;
	public static final int PARAM_CONF_INDEX_Wavenumber_Corr_select = 54;
	public static final int PARAM_CONF_INDEX_COMMON_WAVENUMBER_EN = 57;
	public static final int PARAM_CONF_INDEX_WAVENUMBER_VECTOR_LENGTH = 55;
	
	public static final double adaptiveGainRunTime = 10; //10 ms
	public static final double adaptiveGain_OPT_ADC_REF = 1.35;
	public static final double adaptiveGain_OPT_ADC_FULL_SCALE = 512;
	
	/* Available optical settings used for adaptive gain*/
	//Detector and Current Range
	public static final String[] currentRanges = new String[] {
			"Detector1->1uA",
			"Detector1->2.5uA",
			"Detector1->5uA",
			"Detector1->10uA",
			"Detector1->20uA",
			"Detector1->40uA",
			"Detector1->80uA"};
	//PGA1 gain
	public static final int[] PGA1_values = new int[] {1, 2, 4, 6, 8, 10};

	//PGA2 gain
	public static final int[] PGA2_values = new int[] {1, 2, 4, 6, 8, 10};
	
	public static final String InterSpecPrefix = "_InterSpec_";

	public static final String SpecPrefix = "_Spec_";
	
	public static final int OPEN_LOOP_SAMPLE_FOLDERS_VERSION = 0;
	public static final int CLOSED_LOOP_SAMPLE_FOLDERS_VERSION = 0;
	public static final int UNKNOWN_SAMPLE_FOLDERS_VERSION = 0;
	public static final int ADAPTIVE_GAIN_SAMPLE_FOLDERS_VERSION = 1;
	public static final int RESTORE_DEFAULT_SAMPLE_FOLDERS_VERSION = 2;
	
	public static final int FALLING_NEGATIVE_DIRECTION = 0;
	public static final int RISING_POSITIVE_DIRECTION = 1;
	
	
	public static String getPath(String path)
	{
		return path.replace(Original_APPLICATION_WORKING_DIRECTORY, APPLICATION_WORKING_DIRECTORY);
	}
	
	private p3Constants() {
	}

}
