package sws.p3AppManager_micro.configuration;

import sws.p3AppManager_micro.utils.p3Enumerations.p3AppManagerState;


public class p3RunConfig implements Cloneable{

	
	// input data
	public p3AppManagerState runType;

	public double actFreq;
	public short phaseCorrection;
	public short normalizeSpectrum;
	public int WL_Correction;
	public short selfCorrectionEnable;

	// calibration specific data
	public int noOfOptSamples;

	public boolean openLoop;
	public String waveform;
	public double voltage;
	public String actDirection;

	public int samplingRate;
	public double excitationVoltage;
	public double C2VGain;

	public String detectorAndCurrentRange;
	public int pga1;
	public int pga2;
	public boolean hpfBypass;

	public boolean useInterpolationThreshold;
	public double interpolationThreshold;
	public int interpolationOffset;
	public int interpolationSamples;
	public String interpolationDirection;

	public boolean LaserCalibrationReading;
	public boolean WLCalibrationReading;
	public boolean MethCalibrationReading;

	public boolean skipMethCholoride;
	public double lambda;

	public String sampleName;
	public String resolution;

	public long[] regValues;
	public String[] resolutionFolders;

	public String restoreOption;

	public double frequencyShift;
	public double frequencyStart;
	public double frequencyEnd;
	public double frequencyStep;

	public double targetCap;
	public double resFreq;
	public double qualityFactor;
	public double forwardGain;

	public double phaseStart;
	public double phaseEnd;
	public 	double phaseStep;

	public double phase1;
	public double phase2;
	public double phase3;

	public double gainIncPerc;

	public int waveformType;

	public boolean twoBurstsEnable;

	public String standardCalibratorType;

	// calculated data
	public int noOfRuns;
	public int noOfSlices;
	public int avgShiftBits;
	public double runTime;
	public int noMeasurements;
	public int timeDelay;
	public int apodizationIndex;
	public int zeroPaddingMultiplier;
	
	public int gainIndex;
	public int gainValue;
	public int commonWaveNum;
	public int mode;
	public int burnMode;
	public int[] CalibrationWells;
	
	public String detector_no;

	public int t1;
	public int t2_c1;
	public int t2_c2;
	public int t2_tmax;
	public int delta_t;
	public int lamps_count;
	public int lamp_sel;
	
	public long[] externalApodizationWindow;
	
	public Object clone() throws CloneNotSupportedException {
	    return super.clone();
	}

}