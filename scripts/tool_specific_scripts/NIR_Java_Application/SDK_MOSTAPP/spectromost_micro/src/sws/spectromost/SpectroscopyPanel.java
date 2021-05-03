package sws.spectromost;

import javax.swing.JPanel;
import javax.imageio.ImageIO;

import java.awt.Dimension;
import java.awt.Image;

import javax.swing.ButtonGroup;
import javax.swing.ImageIcon;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JFormattedTextField;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JRadioButton;
import javax.swing.SwingConstants;

import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;

import javax.swing.JButton;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;

import org.jfree.chart.ChartPanel;

import net.miginfocom.swing.MigLayout;
import sws.spectromost.jfreechart.XYLineChart;
import sws.p3AppManager_micro.utils.p3AppManagerException;
import sws.p3AppManager_micro.utils.p3AppManagerNotification;
import sws.p3AppManager_micro.utils.p3AppManagerUtils;
import sws.p3AppManager_micro.utils.p3Constants;
import sws.p3AppManager_micro.utils.p3Enumerations.p3AppManagerStatus;

import javax.swing.event.PopupMenuEvent;
import javax.swing.event.PopupMenuListener;
import javax.swing.filechooser.FileNameExtensionFilter;
import javax.swing.JSeparator;

import java.awt.Font;
import java.awt.Color;

import javax.swing.DefaultComboBoxModel;
import javax.swing.border.LineBorder;

import java.awt.Label;

import org.apache.log4j.Logger;

import java.awt.Component;

@SuppressWarnings("serial")
public class SpectroscopyPanel extends JPanel {

	private static Logger logger = Logger.getLogger(SpectroscopyPanel.class);
	/*
	 * ! Spectroscopy tab
	 */
	public static final String SPECTRUM_REFL_PATH_TEMPLATE = p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH)
			+ File.separatorChar + "Measurement_Perc_{0}.Spectrum";
	public static final String SPECTRUM_ABS_PATH_TEMPLATE = p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH)
			+ File.separatorChar + "Measurement_Abs_{0}.Spectrum";

	public static final String SPECTRUM_FILE_X_AXIS_CM = "x_Axis:Wavenumber (cm-1)";
	public static final String SPECTRUM_FILE_X_AXIS_NM = "x_Axis:Wavelength (nm)";
	public static final String SPECTRUMFILE_Y_AXIS_R_T = "y_Axis:%Reflectance or Transmittance";
	public static final String SPECTRUMFILE_Y_AXIS_A = "y_Axis:Absorbance";

	/*
	 * ! boolean to check whether a capture button was pressed
	 */
	public static boolean capture_Displayed_Spectroscopy = false;

	/*
	 * ! boolean to guard the clearing plots method
	 */
	private static boolean clearingGraphsInProgress = false;

	/*
	 * ! The scan time of the last background reading
	 */
	static String backgroundTime;

	/*
	 * ! The last selected resolution
	 */
	public static String lastResolutionSelected = "";

	/*
	 * ! The last selected opticalGainSettings
	 */
	public static String lastOpticalSettingsSelected = "";

	/*
	 * ! The last selected zero Padding option
	 */
	public static String lastZeroPaddingSelected = "";

	/*
	 * ! The last selected window option
	 */
	public static String lastWindowSelected = "";

	/*
	 * ! The last selected common wavenumber option
	 */
	public static String lastCommonWavenumberSelected = "";

	/*
	 * ! Auto Save Paths for different tabs
	 */
	private String AutoSavePath_Spectroscopy = "";

	/*
	 * GUI fields
	 */
	static Label lbl_moduleID_Spec;
	public static final JCheckBox chb_AutoSave_Spectroscopy = new JCheckBox(
			"Auto-save");
	public static final JCheckBox chb_EnableLinearInterpolation_Spectroscopy = new JCheckBox("Enable linear interpolaion");
	public static JRadioButton rdbtn_Absorbance = new JRadioButton("Abs.");
	public static JRadioButton rdbtn_Refl_Trans = new JRadioButton("%Refl. or Trans.");
	public static JRadioButton rdbtn_nm_Spec = new JRadioButton("nm");
	public static JRadioButton rdbtn_cm_Spec = new JRadioButton("cm\u207B\u00B9");
	public static JRadioButton rdbtn_SingleRun_Spec;
	public static JRadioButton rdbtn_ContRun_Spec;
	static XYLineChart spectroscopyChartReflTrans_NM = null;
	static XYLineChart spectroscopyChartAbs_NM = null;
	static XYLineChart spectroscopyChartReflTrans_CM = null;
	static XYLineChart spectroscopyChartAbs_CM = null;
	static ChartPanel spectroscopyChartPanel = null;
	static JButton btn_Run_Spec;
	static JButton btn_Capture_Spec;
	static JButton btn_ClearGraphs_Spec;
	static JButton btn_LoadGraphs_Spec;
	static JButton btn_Run_Background;
	static JButton btn_SaveGraphs_Spec;
	static JButton btn_Stop_Spec;
	static JButton btn_GainAdjustmentPanel_Spec;
	static JButton btn_WavelengthWavenumberCalibration_Spec;
	static JButton btn_WriteSettings_Spec;
	static JButton btn_RestoreDefault_spec;
	static JButton btn_DeleteOpticalOption_Spec;
	static JComboBox<String> cmb_Apodization_Spec;
	static JComboBox<String> cmb_ZeroPadding_Spec;
	static JComboBox<String> cmb_Optical_Settings_Spec;
	static JComboBox<String> cmb_Wavenumber_No_Of_Points_Spec;
	static JFormattedTextField txt_RunTime_Spec;
	static Component horizontalSpring;
	static Label lbl_FFT_Settings_Spec;
	static Label lbl_advancedSettings_Spec;
	static JSeparator separatorV1_Spec;
	static Label lbl_Apodization_Spec;
	static Label lbl_ZeroPadding_Spec;

	/*
	 * ! Spectrum's axis default values
	 */
	public static final double SPECTRUM_X_NM_MIN = 1285;
	public static final double SPECTRUM_X_NM_MAX = 2615;
	public static final double SPECTRUM_X_CM_MIN = 3700;
	public static final double SPECTRUM_X_CM_MAX = 7795;
	public static final double SPECTRUM_Y_REF_MIN = 90;
	public static final double SPECTRUM_Y_REF_MAX = 110;
	public static final double SPECTRUM_Y_ABS_MIN = 0;
	public static final double SPECTRUM_Y_ABS_MAX = 1.2;
	private static int measurementCount_Spectroscopy = 0;
	private boolean genuineAction = false;

	public SpectroscopyPanel() {
		super();
		this.initialize();
	}

	public void initialize() {
		this.setFont(new Font("Dialog", Font.PLAIN, 12));
		this.setBackground(new Color(176, 196, 222));
		this.setLayout(new MigLayout("", "[90.00:90.00:90.00][45.00:45.00:45.00][45.00:45.00:45.00][30.00:30.00:30.00][30.00:30.00:30.00][30.00:30.00:30.00][90.00:90.00:90.00][90.00:90.00:90.00][90.00:90.00:90.00][90.00:90.00:90.00][grow,fill][][][][][grow,fill]", "[][][][][][][][][][][][][][][][][][25.00:25.00:25.00][20:n][][][][][][][][][][][][grow,fill]"));

		Label lbl_SpectrometerInfo_Spec = new Label("NeoSpectra Micro kit Info");
		lbl_SpectrometerInfo_Spec.setFont(new Font("Dialog", Font.BOLD, 12));
		add(lbl_SpectrometerInfo_Spec, "cell 0 0 6 1");

		Label lbl_SpectrometerID_Spec = new Label("Kit ID");
		lbl_SpectrometerID_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		this.add(lbl_SpectrometerID_Spec, "cell 0 1 2 1,growx");

		lbl_moduleID_Spec = new Label(UserInterface.moduleID);
		lbl_moduleID_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		this.add(lbl_moduleID_Spec, "cell 2 1 2 1,alignx right");

		JSeparator separator1_Spec = new JSeparator();
		add(separator1_Spec, "cell 0 2 6 1,growx");

		Label lbl_MeasurementParams_Spec = new Label("Measurement Parameters");
		lbl_MeasurementParams_Spec.setFont(new Font("Dialog", Font.BOLD, 12));
		this.add(lbl_MeasurementParams_Spec, "cell 0 3 6 1,growx");

		Label lbl_RunTime_Spec = new Label("Scan Time");
		lbl_RunTime_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		this.add(lbl_RunTime_Spec, "cell 0 4,growx");

		txt_RunTime_Spec = new JFormattedTextField();
		txt_RunTime_Spec.setEditable(true);
		txt_RunTime_Spec.setFont(new Font("Dialog", Font.BOLD, 10));
		txt_RunTime_Spec.setText("2");
		txt_RunTime_Spec.setMinimumSize(new Dimension( p3Constants.MAX_WIDTH_OF_FIELD, txt_RunTime_Spec.getPreferredSize().height));
		txt_RunTime_Spec.setMaximumSize(new Dimension( p3Constants.MAX_WIDTH_OF_FIELD, txt_RunTime_Spec.getPreferredSize().height));
		txt_RunTime_Spec.setPreferredSize(new Dimension( p3Constants.MAX_WIDTH_OF_FIELD, txt_RunTime_Spec.getPreferredSize().height));
		this.add(txt_RunTime_Spec, "cell 2 4 3 1,alignx left,growy");

		Label lbl_Sec_Spec = new Label("s");
		lbl_Sec_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		this.add(lbl_Sec_Spec, "cell 5 4,growx");

		Label lbl_Optical_Gain = new Label("Optical Gain Settings");
		lbl_Optical_Gain.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(lbl_Optical_Gain, "cell 0 5,growx");

		cmb_Optical_Settings_Spec = new JComboBox<String>();
		cmb_Optical_Settings_Spec.setEnabled(false);
		cmb_Optical_Settings_Spec.setEditable(false);
		cmb_Optical_Settings_Spec.setFont(new Font("Dialog", Font.BOLD, 10));
		cmb_Optical_Settings_Spec.setMaximumSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_Optical_Settings_Spec.getPreferredSize().height));
		this.add(cmb_Optical_Settings_Spec, "cell 2 5 3 1,growx");

		cmb_Optical_Settings_Spec.addPopupMenuListener(new PopupMenuListener() {
			public void popupMenuCanceled(PopupMenuEvent e) {
			}

			public void popupMenuWillBecomeInvisible(PopupMenuEvent e) {
				InterSpecPanel.cmb_Optical_Settings_Inter_Spec.setSelectedIndex(cmb_Optical_Settings_Spec.getSelectedIndex());
			}

			public void popupMenuWillBecomeVisible(PopupMenuEvent e) {
				UserInterface.displaySpecOpticalSettings();
				UserInterface.displayInterSpecOpticalSettings();
			}
		});

		btn_DeleteOpticalOption_Spec = new JButton("");
		btn_DeleteOpticalOption_Spec.setEnabled(false);
		btn_DeleteOpticalOption_Spec.setVisible(true);
		btn_DeleteOpticalOption_Spec.setToolTipText("Delete the selected optical gain setting");
		btn_DeleteOpticalOption_Spec.setMaximumSize(new Dimension(30, cmb_Optical_Settings_Spec.getPreferredSize().height));
		btn_DeleteOpticalOption_Spec.setMinimumSize(new Dimension(30, cmb_Optical_Settings_Spec.getPreferredSize().height));
		btn_DeleteOpticalOption_Spec.setPreferredSize(new Dimension(30, cmb_Optical_Settings_Spec.getPreferredSize().height));
		btn_DeleteOpticalOption_Spec.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {

				if(cmb_Optical_Settings_Spec.getSelectedItem().toString().equals("Default"))
				{
					JOptionPane.showMessageDialog(null,
							"Default Option cannot be deleted!",
							"Delete optical gain", JOptionPane.OK_OPTION);
					return;
				}

				if(JOptionPane.showConfirmDialog(null, "Are you sure you want to delete " + cmb_Optical_Settings_Spec.getSelectedItem().toString() + " ?", 
						"Delete optical gain", JOptionPane.YES_NO_OPTION) == JOptionPane.NO_OPTION)
				{
					return;
				}

				UserInterface.deleteOpticalSetting(cmb_Optical_Settings_Spec.getSelectedItem().toString());
				UserInterface.displaySpecOpticalSettings();
				UserInterface.displayInterSpecOpticalSettings();
			}
		});
		add(btn_DeleteOpticalOption_Spec, "cell 5 5,grow");

		Label lbl_RunMode_Spec = new Label("Run Mode");
		lbl_RunMode_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(lbl_RunMode_Spec, "cell 0 6,growx");

		ButtonGroup bgRunMode_Spec = new ButtonGroup();

		btn_Run_Background = new JButton("");
		btn_Run_Background.setEnabled(false);
		btn_Run_Background.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_Run_Background.setText("Background");
		btn_Run_Background.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_Run_Background.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_Run_Background.setToolTipText("Run Background");
		btn_Run_Background.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {

				try{

					// Store background reading time
					backgroundTime = String.valueOf(Double.parseDouble(txt_RunTime_Spec.getText()) * 1000);

					// stop checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = true;

					try
					{
						lastZeroPaddingSelected = String.valueOf(cmb_ZeroPadding_Spec.getSelectedIndex());
						
						int commonWavenumberSelection = 0;
						if(chb_EnableLinearInterpolation_Spectroscopy.isSelected())
						{
							commonWavenumberSelection = cmb_Wavenumber_No_Of_Points_Spec.getSelectedIndex() + 1;
						}
						
						lastCommonWavenumberSelected = String.valueOf(commonWavenumberSelection);
						lastWindowSelected = String.valueOf(cmb_Apodization_Spec.getSelectedIndex());
						lastOpticalSettingsSelected = cmb_Optical_Settings_Spec.getSelectedItem().toString();
					}
					catch(Exception ex)
					{
						logger.error(ex.getMessage());
						JOptionPane.showMessageDialog(null,
								"Run failed to start. Module setup failed!",
								"Run NeoSpectra Micro", JOptionPane.OK_OPTION);

						UserInterface.checkDeviceStatusThreadStop = false;
						return;
					}

					UserInterface.progressTime = Double.parseDouble(txt_RunTime_Spec.getText()) * 1000;
					UserInterface.progressPar.setValue(0);

					int commonWavenumberSelection = 0;
					if(chb_EnableLinearInterpolation_Spectroscopy.isSelected())
					{
						commonWavenumberSelection = cmb_Wavenumber_No_Of_Points_Spec.getSelectedIndex() + 1;
					}
					while(UserInterface.acquireDeviceToken() == false);
					p3AppManagerStatus status = UserInterface.applicationManager.runSpec(backgroundTime, "false",
							String.valueOf(cmb_Apodization_Spec.getSelectedIndex()),
							String.valueOf(cmb_ZeroPadding_Spec.getSelectedIndex()),
							UserInterface.getOpticalGainValue((String)cmb_Optical_Settings_Spec.getSelectedItem()),
							String.valueOf(commonWavenumberSelection), Integer.toString(rdbtn_ContRun_Spec.isSelected()? 1:0));

					if (p3AppManagerStatus.NO_ERROR != status) {

						if (p3AppManagerStatus.DEVICE_BUSY_ERROR != status) {
							// resume checking the status of the device
							UserInterface.checkDeviceStatusThreadStop = false;
						}

						JOptionPane.showMessageDialog(null,
								"Run failed to start: " + UserInterface.convertErrorCodesToMessages(status),
								"Run NeoSpectra Micro", JOptionPane.OK_OPTION);


						// resume checking the status of the device
						UserInterface.releaseDeviceToken();
					} else {

						UserInterface.boardReadyRoutine(false);

						UserInterface.statusLabel
						.setText("Run started. Please wait...");
						UserInterface.colorLabel.setForeground(Color.yellow);
						UserInterface.colorLabel.setBackground(Color.yellow);
					}
				}
				catch(Exception ex)
				{
					logger.error(ex.getMessage());
					JOptionPane.showMessageDialog(null,
							"Run failed to start. \n Please make sure that the entries in measurement parameters are valid.",
							"Run NeoSpectra Micro", JOptionPane.OK_OPTION);

				}
				UserInterface.frmMain.revalidate();
				UserInterface.frmMain.repaint();
			}
		});

		rdbtn_SingleRun_Spec = new JRadioButton("Single");
		rdbtn_SingleRun_Spec.setEnabled(false);
		rdbtn_SingleRun_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		rdbtn_SingleRun_Spec.setSelected(true);
		rdbtn_SingleRun_Spec.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				btn_Capture_Spec.setEnabled(false);
			}
		});
		add(rdbtn_SingleRun_Spec, "cell 1 6 2 1,growx");
		bgRunMode_Spec.add(rdbtn_SingleRun_Spec);

		rdbtn_ContRun_Spec = new JRadioButton("Cont.");
		rdbtn_ContRun_Spec.setEnabled(false);
		rdbtn_ContRun_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		rdbtn_ContRun_Spec.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				btn_Capture_Spec.setEnabled(true);
			}
		});
		add(rdbtn_ContRun_Spec, "cell 3 6 3 1,growx");
		bgRunMode_Spec.add(rdbtn_ContRun_Spec);

		this.add(btn_Run_Background, "cell 0 7,growx");

		btn_Run_Spec = new JButton();
		btn_Run_Spec.setEnabled(false);
		btn_Run_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_Run_Spec.setText("Run");
		btn_Run_Spec.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_Run_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_Run_Spec.setToolTipText("Run");
		btn_Run_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				RunSampleSequence();
				try{logger.info("Run_Spec Finished (With PowerUp) - Scan Time = " + String.valueOf(Double.parseDouble(txt_RunTime_Spec.getText()) * 1000));}catch(Exception d){}
			}
		});
		this.add(btn_Run_Spec, "cell 1 7 2 1,growx");
		btn_Stop_Spec = new JButton();
		btn_Stop_Spec.setEnabled(false);
		btn_Stop_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_Stop_Spec.setText("Stop");
		btn_Stop_Spec.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_Stop_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_Stop_Spec.setToolTipText("Stop");
		btn_Stop_Spec.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				UserInterface.stopContinuousRun = true;
				UserInterface.isContinuousRun = false;

				UserInterface.statusLabel.setText("Stopping NeoSpectra Micro kit. Please wait...");
			}
		});
		this.add(btn_Stop_Spec, "cell 3 7 3 1,growx");


		JSeparator separator2_Spec = new JSeparator();
		this.add(separator2_Spec, "cell 0 8 6 1,growx");

		Label lbl_Graphs_Spec = new Label("Plots");
		lbl_Graphs_Spec.setFont(new Font("Dialog", Font.BOLD, 12));
		this.add(lbl_Graphs_Spec, "cell 0 9,growx");

		btn_SaveGraphs_Spec = new JButton("");
		btn_SaveGraphs_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_SaveGraphs_Spec.setText("Save");
		btn_SaveGraphs_Spec.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_SaveGraphs_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_SaveGraphs_Spec.setToolTipText("Save Plots");
		btn_SaveGraphs_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				UserInterface.stopEnablingButtons = true;
				UserInterface.boardReadyRoutine(false);
				btn_ClearGraphs_Spec.setEnabled(false);

				String [] graphs = spectroscopyChartReflTrans_NM.getSeriesKeys();
				int graph_count = graphs.length;
				for(String graph : graphs)
				{
					double[][] dataRefl_NM = spectroscopyChartReflTrans_NM.getGraphData(graph);
					double[][] dataAbs_NM = spectroscopyChartAbs_NM.getGraphData(graph);
					double[][] dataRefl_CM = spectroscopyChartReflTrans_CM.getGraphData(graph);
					double[][] dataAbs_CM = spectroscopyChartAbs_CM.getGraphData(graph);

					if(rdbtn_nm_Spec.isSelected())
					{
						UserInterface.writeGraphFile(dataRefl_NM[0], dataRefl_NM[1], 
								p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_REFL_PATH_TEMPLATE), graph_count), SPECTRUM_FILE_X_AXIS_NM + "\t" + SPECTRUMFILE_Y_AXIS_R_T); 

						UserInterface.writeGraphFile(dataAbs_NM[0], dataAbs_NM[1], 
								p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_ABS_PATH_TEMPLATE), graph_count), SPECTRUM_FILE_X_AXIS_NM + "\t" + SPECTRUMFILE_Y_AXIS_A); 
					}
					else
					{
						UserInterface.writeGraphFile(dataRefl_CM[0], dataRefl_CM[1], 
								p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_REFL_PATH_TEMPLATE), graph_count), SPECTRUM_FILE_X_AXIS_CM + "\t" + SPECTRUMFILE_Y_AXIS_R_T); 

						UserInterface.writeGraphFile(dataAbs_CM[0], dataAbs_CM[1], 
								p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_ABS_PATH_TEMPLATE), graph_count), SPECTRUM_FILE_X_AXIS_CM + "\t" + SPECTRUMFILE_Y_AXIS_A); 
					}
					graph_count--;
				}

				SaveGraphsPanel savePanel = new SaveGraphsPanel(p3Constants.SpecPrefix);
				savePanel.frmSaveGraphs.setVisible(true);
			}
		});

		chb_AutoSave_Spectroscopy.setFont(new Font("Dialog", Font.PLAIN, 10));
		// sets the auto save path
		chb_AutoSave_Spectroscopy.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {

				JFileChooser autoSaveSpec;
				if(UserInterface.defaultDirectorySaveLoad.equals("") || !new File(UserInterface.defaultDirectorySaveLoad).exists())
				{
					autoSaveSpec = new JFileChooser() {
						@Override
						public void approveSelection() {
							File f = getSelectedFile();
							if (!UserInterface.checkEmptyDirectory(f.getAbsolutePath())) {
								int result = JOptionPane.showConfirmDialog(
										this,
										"The chosen directory is not empty. Overwritting might occur, Are you sure you want to proceed?",
										"Directory not empty",
										JOptionPane.YES_NO_OPTION);
								switch (result) {
								case JOptionPane.YES_OPTION:
									super.approveSelection();
									return;
								case JOptionPane.CLOSED_OPTION:
									return;
								case JOptionPane.NO_OPTION:
									return;
								}
							}
							super.approveSelection();
						}
					};
				}
				else
				{
					autoSaveSpec = new JFileChooser(UserInterface.defaultDirectorySaveLoad) {
						@Override
						public void approveSelection() {
							File f = getSelectedFile();
							if (!UserInterface.checkEmptyDirectory(f.getAbsolutePath())) {
								int result = JOptionPane.showConfirmDialog(
										this,
										"The chosen directory is not empty. Overwritting might occur, Are you sure you want to proceed?",
										"Directory not empty",
										JOptionPane.YES_NO_OPTION);
								switch (result) {
								case JOptionPane.YES_OPTION:
									super.approveSelection();
									return;
								case JOptionPane.CLOSED_OPTION:
									return;
								case JOptionPane.NO_OPTION:
									return;
								}
							}
							super.approveSelection();
						}
					};
				}
				autoSaveSpec.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);

				autoSaveSpec.setApproveButtonText("Save");
				autoSaveSpec.setDialogTitle( "Save");

				if (chb_AutoSave_Spectroscopy.isSelected()) {
					if (autoSaveSpec.showOpenDialog(null) == JFileChooser.APPROVE_OPTION) {
						AutoSavePath_Spectroscopy = autoSaveSpec.getSelectedFile().getAbsolutePath();
						UserInterface.defaultDirectorySaveLoad = autoSaveSpec.getSelectedFile().getAbsolutePath();
					} else {
						chb_AutoSave_Spectroscopy.setSelected(false);
					}
				} else {
					AutoSavePath_Spectroscopy = "";
				}
			}
		});

		this.add(chb_AutoSave_Spectroscopy, "cell 1 9 2 1,growx");
		this.add(btn_SaveGraphs_Spec, "cell 0 10,growx");

		btn_LoadGraphs_Spec = new JButton("");
		btn_LoadGraphs_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_LoadGraphs_Spec.setText("Load");
		btn_LoadGraphs_Spec.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_LoadGraphs_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_LoadGraphs_Spec.setToolTipText("Load Plot");
		btn_LoadGraphs_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if (!(capture_Displayed_Spectroscopy)
						&& (measurementCount_Spectroscopy != 0)) {
					measurementCount_Spectroscopy--;
				}
				capture_Displayed_Spectroscopy = false;

				JFileChooser openSpecFile;
				if(UserInterface.defaultDirectorySaveLoad.equals("") || !new File(UserInterface.defaultDirectorySaveLoad).exists())
				{
					openSpecFile = new JFileChooser();
				}
				else
				{
					openSpecFile = new JFileChooser(UserInterface.defaultDirectorySaveLoad);
				}
				openSpecFile.setMultiSelectionEnabled(true);
				openSpecFile.setFileFilter(new FileNameExtensionFilter(
						"Spectrum plot (*.Spectrum)", "Spectrum"));
				if (openSpecFile.showOpenDialog(null) == JFileChooser.APPROVE_OPTION) {
					for(File loadedfile:openSpecFile.getSelectedFiles()){
						double[][] arraysToPlot = UserInterface
								.loadGraphDataFromFile(loadedfile.getAbsolutePath());

						if(arraysToPlot == null)
							return;

						LoadCharts(arraysToPlot);
					}

					UserInterface.defaultDirectorySaveLoad = openSpecFile.getSelectedFile().getParentFile().getAbsolutePath();
				}
			}	
		});
		this.add(btn_LoadGraphs_Spec, "cell 1 10 2 1,growx");

		btn_Capture_Spec = new JButton("Capture");
		btn_Capture_Spec.setEnabled(false);
		btn_Capture_Spec.setToolTipText("Capture Plot");
		btn_Capture_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_Capture_Spec.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_Capture_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_Capture_Spec.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				if (!capture_Displayed_Spectroscopy) {
					capture_Displayed_Spectroscopy = true;
					if ((!AutoSavePath_Spectroscopy.equals("")) && capture_Displayed_Spectroscopy) {
						try {
							p3AppManagerUtils.createDir(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
							File folder = new File(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
							File[] listOfFiles = folder.listFiles();
							BufferedReader reader = null;
							if (rdbtn_Refl_Trans.isSelected()) {
								if (listOfFiles.length != 0) {
									for (int i = 0; i < listOfFiles.length; i++) {
										if (listOfFiles[i].isFile()) {
											if (listOfFiles[i].getAbsolutePath().endsWith(".Spectrum")) {
												p3AppManagerUtils.createDir(AutoSavePath_Spectroscopy);

												reader = new BufferedReader(new FileReader(listOfFiles[i].getAbsolutePath()));

												// Read units line check whether Refl/Abs
												String line = reader.readLine();
												if (line.contains("y_Axis:%Reflectance") || line.contains("y_Axis:%Transmittance")) {
													Files.copy(listOfFiles[i].toPath(), (new File(AutoSavePath_Spectroscopy + File.separatorChar + listOfFiles[i].getName())).toPath(), StandardCopyOption.REPLACE_EXISTING);
												}
											}
										}
									}
								}
							}

							if (rdbtn_Absorbance.isSelected()) {
								if (listOfFiles.length != 0) {
									for (int i = 0; i < listOfFiles.length; i++) {
										if (listOfFiles[i].isFile()) {
											if (listOfFiles[i].getAbsolutePath().endsWith(".Spectrum")) {
												p3AppManagerUtils.createDir(AutoSavePath_Spectroscopy);
												reader = new BufferedReader( new FileReader( listOfFiles[i].getAbsolutePath()));

												// Read units line check whether Refl/Abs
												String line = reader.readLine();
												if (line.contains("y_Axis:Absorbance")) {
													Files.copy(listOfFiles[i].toPath(), (new File(AutoSavePath_Spectroscopy + File.separatorChar + listOfFiles[i].getName())).toPath(), StandardCopyOption.REPLACE_EXISTING);
												}
											}
										}
									}
								}

							}
						} catch (Exception ex) {
						}
					}
				}
			}
		});
		this.add(btn_Capture_Spec, "cell 1 11 2 1,growx");

		btn_ClearGraphs_Spec = new JButton("");
		btn_ClearGraphs_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_ClearGraphs_Spec.setText("Clear");
		btn_ClearGraphs_Spec.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_ClearGraphs_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_ClearGraphs_Spec.setToolTipText("Clear Plots");
		btn_ClearGraphs_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {

				clearingGraphsInProgress = true;

				p3AppManagerUtils.createDir(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
				File folder = new File(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
				File[] listOfFiles = folder.listFiles();
				for (File file : listOfFiles) {
					if (file.getAbsolutePath().endsWith(".Spectrum")) {
						file.delete();
					}
				}

				// Plot fake data to let the panel appeared
				spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "", "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { new double[] { SPECTRUM_X_NM_MIN }, new double[] { SPECTRUM_Y_REF_MIN }}, SPECTRUM_X_NM_MIN, SPECTRUM_X_NM_MAX, SPECTRUM_Y_REF_MIN, SPECTRUM_Y_REF_MAX);
				spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "", "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { new double[] { SPECTRUM_X_CM_MIN }, new double[] { SPECTRUM_Y_REF_MIN } }, SPECTRUM_X_CM_MIN, SPECTRUM_X_CM_MAX, SPECTRUM_Y_REF_MIN, SPECTRUM_Y_REF_MAX);

				// Plot fake data to let the panel appeared
				spectroscopyChartAbs_NM = new XYLineChart("Spectrum", "", "Wavelength (nm)", "Absorbance units", new double[][] { new double[] { SPECTRUM_X_NM_MIN }, new double[] { SPECTRUM_Y_ABS_MIN } }, SPECTRUM_X_NM_MIN, SPECTRUM_X_NM_MAX, SPECTRUM_Y_ABS_MIN, SPECTRUM_Y_ABS_MAX);
				spectroscopyChartAbs_CM = new XYLineChart("Spectrum", "", "Wavenumber (cm -\u00B9)", "Absorbance units", new double[][] { new double[] { SPECTRUM_X_CM_MIN }, new double[] { SPECTRUM_Y_ABS_MIN } }, SPECTRUM_X_CM_MIN, SPECTRUM_X_CM_MAX, SPECTRUM_Y_ABS_MIN, SPECTRUM_Y_ABS_MAX);

				remove(spectroscopyChartPanel);

				spectroscopyChartPanel = null;

				if(rdbtn_nm_Spec.isSelected())
				{
					spectroscopyChartPanel = spectroscopyChartReflTrans_NM.getChartPanel();
				}
				else
				{
					spectroscopyChartPanel = spectroscopyChartReflTrans_CM.getChartPanel();
				}
				spectroscopyChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));

				//Remove legend of the fake data plotted to chart
				spectroscopyChartPanel.getChart().removeLegend();

				spectroscopyChartPanel.repaint();

				spectroscopyChartPanel.setPreferredSize(new Dimension( (int) spectroscopyChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

				add(spectroscopyChartPanel, "cell 6 0 10 21,grow");

				measurementCount_Spectroscopy = 0;

				rdbtn_Refl_Trans.setSelected(true);
				rdbtn_Absorbance.setSelected(false);
				AutoSavePath_Spectroscopy = "";
				if (chb_AutoSave_Spectroscopy.isSelected()) {
					chb_AutoSave_Spectroscopy.setSelected(false);
					JOptionPane.showMessageDialog(
							null,
							"Auto-save is disabled, please rechoose saving directory",
							"Auto-save disabled!",
							JOptionPane.OK_OPTION);
				}

				UserInterface.frmMain.revalidate();
				UserInterface.frmMain.repaint();

				clearingGraphsInProgress = false;
			}
		});
		this.add(btn_ClearGraphs_Spec, "cell 3 10 3 1,growx");

		JSeparator separator_2 = new JSeparator();
		add(separator_2, "cell 0 12 6 1,growx");

		ButtonGroup bgDataDisplay_X_Axis = new ButtonGroup();

		Label lblDataDisplay = new Label("Data Display");
		lblDataDisplay.setFont(new Font("Dialog", Font.BOLD, 12));
		this.add(lblDataDisplay, "cell 0 15,growx");
		
		Label lbl_CommonWVN_Spec = new Label("Number of Data Points");
		lbl_CommonWVN_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(lbl_CommonWVN_Spec, "cell 0 17,growx");

		cmb_Wavenumber_No_Of_Points_Spec = new JComboBox<String>();
		cmb_Wavenumber_No_Of_Points_Spec.setEnabled(false);
		cmb_Wavenumber_No_Of_Points_Spec.setFont(new Font("Dialog", Font.BOLD, 10));
		cmb_Wavenumber_No_Of_Points_Spec.setMaximumSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_Wavenumber_No_Of_Points_Spec.getPreferredSize().height));
		cmb_Wavenumber_No_Of_Points_Spec.addPopupMenuListener(new PopupMenuListener() {
			public void popupMenuCanceled(PopupMenuEvent e) {
			}

			public void popupMenuWillBecomeInvisible(PopupMenuEvent e) {
				InterSpecPanel.cmb_Wavenumber_No_Of_Points_Inter_Spec.setSelectedIndex(cmb_Wavenumber_No_Of_Points_Spec.getSelectedIndex());
			
			}

			public void popupMenuWillBecomeVisible(PopupMenuEvent e) {
			}
		});
		add(cmb_Wavenumber_No_Of_Points_Spec, "cell 2 17 3 1,growx");

		chb_EnableLinearInterpolation_Spectroscopy.setFont(new Font("Dialog", Font.PLAIN, 10));
		this.add(chb_EnableLinearInterpolation_Spectroscopy, "cell 0 16 3 1,growx");
		chb_EnableLinearInterpolation_Spectroscopy.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {

				InterSpecPanel.chb_EnableLinearInterpolation_Inter_Spec.setSelected(chb_EnableLinearInterpolation_Spectroscopy.isSelected());
				
				if(chb_EnableLinearInterpolation_Spectroscopy.isSelected())
				{
					cmb_Wavenumber_No_Of_Points_Spec.setEnabled(true);
					UserInterface.displayCommonWVN();
					cmb_Wavenumber_No_Of_Points_Spec.setSelectedIndex(4);
					
					InterSpecPanel.cmb_Wavenumber_No_Of_Points_Inter_Spec.setEnabled(true);
					InterSpecPanel.cmb_Wavenumber_No_Of_Points_Inter_Spec.setSelectedIndex(4);
				}
				else
				{
					cmb_Wavenumber_No_Of_Points_Spec.setEnabled(false);
					cmb_Wavenumber_No_Of_Points_Spec.removeAllItems();
					
					InterSpecPanel.cmb_Wavenumber_No_Of_Points_Inter_Spec.setEnabled(false);
					InterSpecPanel.cmb_Wavenumber_No_Of_Points_Inter_Spec.removeAllItems();
					
				}
				
			}
		});
		
		ButtonGroup bgDataDisplay_Y_Axis = new ButtonGroup();
		rdbtn_nm_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {

				remove(spectroscopyChartPanel);

				spectroscopyChartPanel = null;

				if(rdbtn_Absorbance.isSelected())
				{
					spectroscopyChartPanel = spectroscopyChartAbs_NM.getChartPanel();
				}
				else
				{
					spectroscopyChartPanel = spectroscopyChartReflTrans_NM.getChartPanel();
				}
				spectroscopyChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));

				//Remove legend of the fake data plotted to chart if no valid data was plotted
				if(measurementCount_Spectroscopy == 0)
				{
					spectroscopyChartPanel.getChart().removeLegend();
				}

				spectroscopyChartPanel.repaint();

				spectroscopyChartPanel.setPreferredSize(new Dimension( (int) spectroscopyChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

				add(spectroscopyChartPanel, "cell 6 0 10 21,grow");
				revalidate();
				repaint();
			}
		});

		Label lblXaxis_Spec = new Label("X-Axis:");
		lblXaxis_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(lblXaxis_Spec, "cell 0 18,growx,aligny top");
		Label lblYaxis_Spec = new Label("Y-Axis:");
		lblYaxis_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		this.add(lblYaxis_Spec, "cell 0 19,growx,aligny top");
		rdbtn_nm_Spec.setSelected(true);
		rdbtn_nm_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(rdbtn_nm_Spec, "flowx,cell 1 18 2 1,aligny top");
		bgDataDisplay_X_Axis.add(rdbtn_nm_Spec);
		rdbtn_cm_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {

				remove(spectroscopyChartPanel);

				spectroscopyChartPanel = null;

				if(rdbtn_Absorbance.isSelected())
				{
					spectroscopyChartPanel = spectroscopyChartAbs_CM.getChartPanel();
				}
				else
				{
					spectroscopyChartPanel = spectroscopyChartReflTrans_CM.getChartPanel();
				}
				spectroscopyChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));

				//Remove legend of the fake data plotted to chart if no valid data was plotted
				if(measurementCount_Spectroscopy == 0)
				{
					spectroscopyChartPanel.getChart().removeLegend();
				}

				spectroscopyChartPanel.repaint();

				spectroscopyChartPanel.setPreferredSize(new Dimension( (int) spectroscopyChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

				add(spectroscopyChartPanel, "cell 6 0 10 21,grow");
				revalidate();
				repaint();
			}
		});

		rdbtn_cm_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(rdbtn_cm_Spec, "cell 3 18 3 1,aligny top");
		bgDataDisplay_X_Axis.add(rdbtn_cm_Spec);
		rdbtn_Refl_Trans.setFont(new Font("Dialog", Font.PLAIN, 10));

		this.add(rdbtn_Refl_Trans, "flowx,cell 1 19 2 1,aligny top");
		rdbtn_Refl_Trans.setSelected(true);

		rdbtn_Refl_Trans.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				remove(spectroscopyChartPanel);

				spectroscopyChartPanel = null;

				if(rdbtn_nm_Spec.isSelected())
				{
					spectroscopyChartPanel = spectroscopyChartReflTrans_NM.getChartPanel();
				}
				else
				{
					spectroscopyChartPanel = spectroscopyChartReflTrans_CM.getChartPanel();
				}
				spectroscopyChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));

				spectroscopyChartPanel.repaint();
				spectroscopyChartPanel.revalidate();

				spectroscopyChartPanel.setPreferredSize(new Dimension( (int) spectroscopyChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

				add(spectroscopyChartPanel, "cell 6 0 10 21,grow");
				revalidate();
				repaint();

			}
		});
		bgDataDisplay_Y_Axis.add(rdbtn_Refl_Trans);
		rdbtn_Absorbance.setFont(new Font("Dialog", Font.PLAIN, 10));

		this.add(rdbtn_Absorbance, "cell 3 19 3 1,aligny top");
		rdbtn_Absorbance.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {

				remove(spectroscopyChartPanel);

				spectroscopyChartPanel = null;

				if(rdbtn_nm_Spec.isSelected())
				{
					spectroscopyChartPanel = spectroscopyChartAbs_NM.getChartPanel();
				}
				else
				{
					spectroscopyChartPanel = spectroscopyChartAbs_CM.getChartPanel();
				}
				spectroscopyChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));

				//Remove legend of the fake data plotted to chart if no valid data was plotted
				if(measurementCount_Spectroscopy == 0)
				{
					spectroscopyChartPanel.getChart().removeLegend();
				}

				spectroscopyChartPanel.repaint();

				spectroscopyChartPanel.setPreferredSize(new Dimension( (int) spectroscopyChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

				add(spectroscopyChartPanel, "cell 6 0 10 21,grow");
				revalidate();
				repaint();
			}
		});
		bgDataDisplay_Y_Axis.add(rdbtn_Absorbance);


		JSeparator separator4_Spec = new JSeparator();
		this.add(separator4_Spec, "cell 0 20 6 1,growx");


		lbl_advancedSettings_Spec = new Label("Advanced Settings");
		lbl_advancedSettings_Spec.setVisible(false);
		lbl_advancedSettings_Spec.setFont(new Font("Dialog", Font.BOLD, 12));
		this.add(lbl_advancedSettings_Spec, "cell 6 22 2 1,growx");

		btn_WavelengthWavenumberCalibration_Spec = new JButton("");
		btn_WavelengthWavenumberCalibration_Spec.setVisible(false);
		btn_WavelengthWavenumberCalibration_Spec.setEnabled(false);
		btn_WavelengthWavenumberCalibration_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_WavelengthWavenumberCalibration_Spec.setText("<html><center>Wavelength Correction</center></html>");
		btn_WavelengthWavenumberCalibration_Spec.setVerticalTextPosition(SwingConstants.CENTER);
		btn_WavelengthWavenumberCalibration_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_WavelengthWavenumberCalibration_Spec.setToolTipText("Wavelength & Wavenumber Correction");

		btn_WavelengthWavenumberCalibration_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				UserInterface.stopEnablingButtons = true;
				UserInterface.boardReadyRoutine(false);
				try{
					int commonWavenumberSelection = 0;
					if(chb_EnableLinearInterpolation_Spectroscopy.isSelected())
					{
						commonWavenumberSelection = cmb_Wavenumber_No_Of_Points_Spec.getSelectedIndex() + 1;
					}
					
					WL_WN_CalibrationPanel calibratoinPanel = new WL_WN_CalibrationPanel(
							String.valueOf(Double.parseDouble(txt_RunTime_Spec.getText()) * 1000),
							"",
							UserInterface.getOpticalGainValue((String)cmb_Optical_Settings_Spec.getSelectedItem()),
							String.valueOf(cmb_Apodization_Spec.getSelectedIndex()),
							String.valueOf(cmb_ZeroPadding_Spec.getSelectedIndex()),
							String.valueOf(commonWavenumberSelection),
							p3Constants.SpecPrefix);
					calibratoinPanel.frmWL_Calib.setVisible(true);
				}
				catch (Exception e)
				{
					UserInterface.stopEnablingButtons = false;
					UserInterface.boardReadyRoutine(true);
					JOptionPane.showMessageDialog(null,
							"Failed to start. \n Please make sure that entries in measurement parameters are valid.",
							"Wavelength & Wavenumber Correction", JOptionPane.OK_OPTION);
				}
			}
		});

		this.add(btn_WavelengthWavenumberCalibration_Spec, "cell 8 23 2 1,grow");

		lbl_FFT_Settings_Spec = new Label("FFT Settings");
		lbl_FFT_Settings_Spec.setVisible(false);
		lbl_FFT_Settings_Spec.setFont(new Font("Dialog", Font.BOLD, 12));
		this.add(lbl_FFT_Settings_Spec, "cell 12 22,growx");

		btn_GainAdjustmentPanel_Spec = new JButton("");
		btn_GainAdjustmentPanel_Spec.setVisible(false);
		btn_GainAdjustmentPanel_Spec.setEnabled(false);
		btn_GainAdjustmentPanel_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_GainAdjustmentPanel_Spec.setText("<html><center>Add New Optical Gain Setting</center></html>");
		btn_GainAdjustmentPanel_Spec.setVerticalTextPosition(SwingConstants.CENTER);
		btn_GainAdjustmentPanel_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_GainAdjustmentPanel_Spec.setToolTipText("Add new optical gain setting");

		btn_GainAdjustmentPanel_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				Object[] options = {"OK"};
				int input = JOptionPane.showOptionDialog(null,
						"Please make sure that your sample is in the light path!",
						"Warning!",
						JOptionPane.OK_OPTION,
						JOptionPane.WARNING_MESSAGE,
						null,
						options,
						options[0]);
				if (input == JOptionPane.CLOSED_OPTION){
					return;
				}else if(input == JOptionPane.OK_OPTION){
					UserInterface.GainAdjustment();
				}
			}
		});
		this.add(btn_GainAdjustmentPanel_Spec, "cell 6 23 2 1,grow");

		lbl_Apodization_Spec = new Label("Apodization Function");
		lbl_Apodization_Spec.setVisible(false);
		lbl_Apodization_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(lbl_Apodization_Spec, "cell 12 23,growx");

		separatorV1_Spec = new JSeparator();
		separatorV1_Spec.setVisible(false);
		separatorV1_Spec.setOrientation(SwingConstants.VERTICAL);
		add(separatorV1_Spec, "cell 11 22 1 5,growy");


		btn_WriteSettings_Spec = new JButton("");
		btn_WriteSettings_Spec.setVisible(false);
		btn_WriteSettings_Spec.setEnabled(false);
		btn_WriteSettings_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_WriteSettings_Spec.setText("Burn Settings");
		btn_WriteSettings_Spec.setVerticalTextPosition(SwingConstants.CENTER);
		btn_WriteSettings_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_WriteSettings_Spec.setToolTipText("Burn current settings in kit's ROM");
		btn_WriteSettings_Spec.addActionListener(new ActionListener() {																			
			public void actionPerformed(ActionEvent arg0) {

				UserInterface.burnSequence();

			}
		});
		this.add(btn_WriteSettings_Spec, "cell 6 24 2 1,growx");

		cmb_Apodization_Spec = new JComboBox<String>();
		cmb_Apodization_Spec.setEnabled(false);
		cmb_Apodization_Spec.setVisible(false);
		cmb_Apodization_Spec.setFont(new Font("Dialog", Font.BOLD, 10));
		cmb_Apodization_Spec.setModel(new DefaultComboBoxModel<String>(p3Constants.apodizationOptions));
		cmb_Apodization_Spec.setMaximumSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_Apodization_Spec.getPreferredSize().height));
		cmb_Apodization_Spec.setMinimumSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_Apodization_Spec.getPreferredSize().height));
		cmb_Apodization_Spec.setPreferredSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_Apodization_Spec.getPreferredSize().height));

		this.add(cmb_Apodization_Spec, "flowx,cell 13 23 3 1,growx");

		cmb_Apodization_Spec.addPopupMenuListener(new PopupMenuListener() {
			public void popupMenuCanceled(PopupMenuEvent e) {
			}

			public void popupMenuWillBecomeInvisible(PopupMenuEvent e) {
				InterSpecPanel.cmb_Apodization_Inter_Spec.setSelectedIndex(cmb_Apodization_Spec.getSelectedIndex());
			}

			public void popupMenuWillBecomeVisible(PopupMenuEvent e) {
				cmb_Apodization_Spec.setModel(new DefaultComboBoxModel<String>(p3Constants.apodizationOptions));
				genuineAction = true;
			}
		});
		cmb_Apodization_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				File loadedfile = null;
				if(genuineAction)
				{
					genuineAction = false;
					long[] apodizationWindowCoefficients = null;
					
					if(cmb_Apodization_Spec.getSelectedItem().toString().startsWith("External Function"))
					{
						JFileChooser openWindowFile;
						if(UserInterface.defaultDirectorySaveLoad.equals("") || !new File(UserInterface.defaultDirectorySaveLoad).exists())
						{
							openWindowFile = new JFileChooser();
						}
						else
						{
							openWindowFile = new JFileChooser(UserInterface.defaultDirectorySaveLoad);
						}
						
						openWindowFile.setMultiSelectionEnabled(false);
						openWindowFile.setFileFilter(new FileNameExtensionFilter(
								"Apodization window coefficients (*.txt)", "txt"));
						if (openWindowFile.showOpenDialog(null) == JFileChooser.APPROVE_OPTION) {

							loadedfile = openWindowFile.getSelectedFile();

							apodizationWindowCoefficients = p3AppManagerUtils.loadLongDataFile(loadedfile.getAbsolutePath());

							long sum = 0;
							for(long d : apodizationWindowCoefficients)
							    sum += d;
							
							if(apodizationWindowCoefficients == null || apodizationWindowCoefficients.length > p3Constants.EXTERNAL_WINDOW_MAX_LENGTH || apodizationWindowCoefficients.length == 0 || sum == 0)
							{
								cmb_Apodization_Spec.setSelectedIndex(0);
								InterSpecPanel.cmb_Apodization_Inter_Spec.setSelectedIndex(cmb_Apodization_Spec.getSelectedIndex());
								JOptionPane.showMessageDialog(
										null,
										"Invalid apodization window. \nPlease select a valid window coefficients between 1~20 coefficients",
										"Apodization window", JOptionPane.OK_OPTION);
								apodizationWindowCoefficients = null;
								return;
							}

							UserInterface.defaultDirectorySaveLoad = openWindowFile.getSelectedFile().getParentFile().getAbsolutePath();
						}

						UserInterface.applicationManager.setExternalApodizationWindow(apodizationWindowCoefficients);
						lastWindowSelected = "-1";
					}
				}
				if(loadedfile !=null){
					String fileName = loadedfile.getName().replace(".txt", "");
					
					cmb_Apodization_Spec.removeItemAt(cmb_Apodization_Spec.getItemCount() - 1);
					cmb_Apodization_Spec.addItem(fileName);
					cmb_Apodization_Spec.setSelectedIndex(cmb_Apodization_Spec.getItemCount() - 1);
					
					InterSpecPanel.cmb_Apodization_Inter_Spec.removeItemAt(InterSpecPanel.cmb_Apodization_Inter_Spec.getItemCount() - 1);
					InterSpecPanel.cmb_Apodization_Inter_Spec.addItem(fileName);
					InterSpecPanel.cmb_Apodization_Inter_Spec.setSelectedIndex(InterSpecPanel.cmb_Apodization_Inter_Spec.getItemCount() - 1);
				}else{
					InterSpecPanel.cmb_Apodization_Inter_Spec.setSelectedIndex(cmb_Apodization_Spec.getSelectedIndex());
				}
				
			}
		});
		
		JLabel lbl_logo_Spec = new JLabel("");
		lbl_logo_Spec.setFont(new Font("Dialog", Font.BOLD, 10));
		lbl_logo_Spec.setForeground(Color.WHITE);
		lbl_logo_Spec.setBackground(Color.WHITE);
		this.add(lbl_logo_Spec, "flowx,cell 0 22 6 3,alignx left,aligny bottom");

		// Plot fake data to let the panel appeared
		spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "", "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { new double[] { SPECTRUM_X_NM_MIN }, new double[] { SPECTRUM_Y_REF_MIN } }, SPECTRUM_X_NM_MIN, SPECTRUM_X_NM_MAX, SPECTRUM_Y_REF_MIN, SPECTRUM_Y_REF_MAX);
		spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "", "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { new double[] { SPECTRUM_X_CM_MIN }, new double[] { SPECTRUM_Y_REF_MIN } }, SPECTRUM_X_CM_MIN, SPECTRUM_X_CM_MAX, SPECTRUM_Y_REF_MIN, SPECTRUM_Y_REF_MAX);

		// Plot fake data to let the panel appeared
		spectroscopyChartAbs_NM = new XYLineChart("Spectrum", "", "Wavelength (nm)", "Absorbance units", new double[][] { new double[] { SPECTRUM_X_NM_MIN }, new double[] { SPECTRUM_Y_ABS_MIN } }, SPECTRUM_X_NM_MIN, SPECTRUM_X_NM_MAX, SPECTRUM_Y_ABS_MIN, SPECTRUM_Y_ABS_MAX);
		spectroscopyChartAbs_CM = new XYLineChart("Spectrum", "", "Wavenumber (cm -\u00B9)", "Absorbance units", new double[][] { new double[] { SPECTRUM_X_CM_MIN }, new double[] { SPECTRUM_Y_ABS_MIN } }, SPECTRUM_X_CM_MIN, SPECTRUM_X_CM_MAX, SPECTRUM_Y_ABS_MIN, SPECTRUM_Y_ABS_MAX);

		spectroscopyChartPanel = spectroscopyChartReflTrans_NM.getChartPanel();

		spectroscopyChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));

		//Remove legend of the fake data plotted to chart
		spectroscopyChartPanel.getChart().removeLegend();

		spectroscopyChartPanel.setPreferredSize(new Dimension( (int) spectroscopyChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

		this.add(spectroscopyChartPanel, "cell 6 0 10 21,grow");

		lbl_ZeroPadding_Spec = new Label("Zero Padding");
		lbl_ZeroPadding_Spec.setVisible(false);
		lbl_ZeroPadding_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(lbl_ZeroPadding_Spec, "cell 12 24,growx");


		cmb_ZeroPadding_Spec = new JComboBox<String>();
		cmb_ZeroPadding_Spec.setEnabled(false);
		cmb_ZeroPadding_Spec.setVisible(false);
		cmb_ZeroPadding_Spec.setFont(new Font("Dialog", Font.BOLD, 10));
		for(int f = 0; f < p3Constants.paddingOptions.length; f++)
		{
			cmb_ZeroPadding_Spec.addItem(p3Constants.paddingOptions[f] + " N");
		}
		cmb_ZeroPadding_Spec.setMaximumSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_ZeroPadding_Spec.getPreferredSize().height));
		this.add(cmb_ZeroPadding_Spec, "flowx,cell 13 24 3 1,growx");

		btn_RestoreDefault_spec = new JButton("Restore Default Settings");
		btn_RestoreDefault_spec.setVisible(false);
		btn_RestoreDefault_spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_RestoreDefault_spec.setEnabled(false);
		btn_RestoreDefault_spec.setToolTipText("Restore Default Settings");
		btn_RestoreDefault_spec.addActionListener(new ActionListener() {																			
			public void actionPerformed(ActionEvent arg0) {
				UserInterface.restoreSequence();
			}
		});
		add(btn_RestoreDefault_spec, "cell 8 24 2 1,growx");

		cmb_ZeroPadding_Spec.addPopupMenuListener(new PopupMenuListener() {
			public void popupMenuCanceled(PopupMenuEvent e) {
			}

			public void popupMenuWillBecomeInvisible(PopupMenuEvent e) {
				InterSpecPanel.cmb_ZeroPadding_Inter_Spec.setSelectedIndex(cmb_ZeroPadding_Spec.getSelectedIndex());
			}

			public void popupMenuWillBecomeVisible(PopupMenuEvent e) {
			}
		});

		try {
			File imgFile = new File(UserInterface.DELETE_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SCALE, -1, Image.SCALE_SMOOTH);
				btn_ClearGraphs_Spec.setIcon(new ImageIcon(img));
			} else {
				JOptionPane.showMessageDialog(null,
						"Clear plots image doesn't exist.", "Loading image",
						JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(null,
					"Error loading  plots image:" + ex.getMessage(),
					"Loading image", JOptionPane.OK_OPTION);
		}
		try {
			File imgFile = new File(UserInterface.DELETE_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SMALL_SCALE, -1, Image.SCALE_SMOOTH);
				btn_DeleteOpticalOption_Spec.setIcon(new ImageIcon(img));
			} else {
				JOptionPane.showMessageDialog(null,
						"Delete optical gain image doesn't exist.", "Loading image",
						JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(null,
					"Error loading  plots image:" + ex.getMessage(),
					"Loading image", JOptionPane.OK_OPTION);
		}
		try {
			File imgFile = new File(UserInterface.SAVE_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SCALE, -1, Image.SCALE_SMOOTH);
				btn_SaveGraphs_Spec.setIcon(new ImageIcon(img));
			} else {
				JOptionPane.showMessageDialog(null,
						"Save plots image doesn't exist.", "Loading image",
						JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(null,
					"Error loading save plots image:" + ex.getMessage(),
					"Loading image", JOptionPane.OK_OPTION);
		}
		try {
			File imgFile = new File(UserInterface.LOAD_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SCALE, -1, Image.SCALE_SMOOTH);
				btn_LoadGraphs_Spec.setIcon(new ImageIcon(img));
			} else {
				JOptionPane.showMessageDialog(null,
						"Load plots image doesn't exist.", "Loading image",
						JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(null,
					"Error loading load plots image:" + ex.getMessage(),
					"Loading image", JOptionPane.OK_OPTION);
		}
		try {
			File imgFile = new File(UserInterface.CAPTURE_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SCALE, -1, Image.SCALE_SMOOTH);
				btn_Capture_Spec.setIcon(new ImageIcon(img));
			} else {
				JOptionPane.showMessageDialog(null,
						"Capture image doesn't exist.", "Loading image",
						JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(null,
					"Error loading capture image:" + ex.getMessage(),
					"Loading image", JOptionPane.OK_OPTION);
		}
		try {
			File imgFile = new File(UserInterface.LOGO_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance((int)(p3Constants.MAX_WIDTH_OF_FIELD * 2.4), (int)(p3Constants.MAX_WIDTH_OF_FIELD * 0.6) , Image.SCALE_SMOOTH);
				lbl_logo_Spec.setHorizontalTextPosition(SwingConstants.LEFT);
				lbl_logo_Spec.setText("   ");
				lbl_logo_Spec.setIcon(new ImageIcon(img));
			} else {
				JOptionPane.showMessageDialog(null,
						"Logo image doesn't exist.", "Loading image",
						JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(null,
					"Error loading run image:" + ex.getMessage(),
					"Loading image", JOptionPane.OK_OPTION);
		}
		try {
			File imgFile = new File(UserInterface.RUN_BG_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SCALE, -1, Image.SCALE_SMOOTH);
				btn_Run_Background.setIcon(new ImageIcon(img));
			} else {
				JOptionPane.showMessageDialog(null, "Run image doesn't exist.",
						"Loading image", JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(null,
					"Error loading run image:" + ex.getMessage(),
					"Loading image", JOptionPane.OK_OPTION);
		}
		try {
			File imgFile = new File(UserInterface.RUN_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SCALE, -1, Image.SCALE_SMOOTH);
				btn_Run_Spec.setIcon(new ImageIcon(img));
			} else {
				JOptionPane.showMessageDialog(null, "Run image doesn't exist.",
						"Loading image", JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(null,
					"Error loading run image:" + ex.getMessage(),
					"Loading image", JOptionPane.OK_OPTION);
		}
		try {
			File imgFile = new File(UserInterface.STOP_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SCALE, -1, Image.SCALE_SMOOTH);
				btn_Stop_Spec.setIcon(new ImageIcon(img));
			} else {
				JOptionPane.showMessageDialog(null,
						"Stop image doesn't exist.", "Loading image",
						JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(null,
					"Error loading run image:" + ex.getMessage(),
					"Loading image", JOptionPane.OK_OPTION);
		}
	}

	public void update(Object arg1) {
		if (arg1 instanceof p3AppManagerNotification) {
			p3AppManagerNotification resp = (p3AppManagerNotification) arg1;
			
			switch (resp.getAction()) {
			case 2: // Spectroscopy background run

				UserInterface.progressTime = -1;
				UserInterface.progressPar.setValue(100);

				if (resp.getStatus() == 0) {
					UserInterface.statusLabel.setText("Background measurement completed successfully.");
					UserInterface.colorLabel.setForeground(Color.green.darker());
					UserInterface.colorLabel.setBackground(Color.green.darker());
					UserInterface.boardRestarted = false;

				} else {

					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

					JOptionPane.showMessageDialog(
							null,
							UserInterface.convertErrorCodesToMessages(resp.getStatus()), "Run NeoSpectra Micro",
							JOptionPane.OK_OPTION);

				}

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();
				UserInterface.checkDeviceStatusThreadStop = false;
				UserInterface.boardReadyRoutine(true);

				break;

			case 3: // Spectroscopy sample Run
				logger.info("Run_Spec Finished");
				if (!(UserInterface.stopContinuousRun)) {
					if (resp.getStatus() == 0) {
						logger.info("Get Data of Run_Spec Started");
						double[][] data = UserInterface.applicationManager.getSpecData();
						logger.info("Get Data of Run_Spec Finished");

						double transmittanceError = UserInterface.applicationManager.getErrorData();

						double[] xArray = UserInterface.switch_NM_CM(data[p3Constants.WAVENUMBER_INDEX]);
						double[] TArray = UserInterface.convertDataToT(data[p3Constants.POWER_SPECTRAL_DENSITY_INDEX]);
						for(int i = 0; i < TArray.length; i++)
						{
							TArray[i] = TArray[i] / transmittanceError;
						}
						double[] Absorbance = UserInterface.convertRefltoAbs(TArray);

						boolean	clearedGraph = false; 


						if(!chb_EnableLinearInterpolation_Spectroscopy.isSelected())
						{
							String str;
							
							str = String.valueOf((int)data[4][0]);
							str = str + " pts";

							InterSpecPanel.cmb_Wavenumber_No_Of_Points_Inter_Spec.removeAllItems();
							SpectroscopyPanel.cmb_Wavenumber_No_Of_Points_Spec.removeAllItems();
							
							SpectroscopyPanel.cmb_Wavenumber_No_Of_Points_Spec.addItem(str);
							InterSpecPanel.cmb_Wavenumber_No_Of_Points_Inter_Spec.addItem(str);
							
						}
				
						if(clearingGraphsInProgress == false)
						{
							//Clear plots are not running now

							try {
								if(measurementCount_Spectroscopy == 0)
									clearedGraph = true;
								// Automatically overwrite last plot
								if (!(capture_Displayed_Spectroscopy) && (measurementCount_Spectroscopy != 0)) {
									measurementCount_Spectroscopy--;
								}
								// delete last displayed Absorbance/Reflectance
								// chart if not
								// capture
								measurementCount_Spectroscopy++;
								if (measurementCount_Spectroscopy == 1 && clearedGraph) {
									spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { xArray, TArray }, true);
									spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { data[p3Constants.WAVENUMBER_INDEX], TArray }, true);

									spectroscopyChartAbs_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "Absorbance units", new double[][] { xArray, Absorbance }, true);
									spectroscopyChartAbs_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "Absorbance units", new double[][] { data[p3Constants.WAVENUMBER_INDEX], Absorbance }, true);

									clearedGraph = false;
								} else {
									spectroscopyChartReflTrans_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xArray, TArray }, true);
									spectroscopyChartReflTrans_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { data[p3Constants.WAVENUMBER_INDEX], TArray }, true);

									spectroscopyChartAbs_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xArray, Absorbance }, true);
									spectroscopyChartAbs_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { data[p3Constants.WAVENUMBER_INDEX], Absorbance }, true);
								}

								this.remove(spectroscopyChartPanel);
								spectroscopyChartPanel = null;

								// display selected plot.
								if (rdbtn_Refl_Trans.isSelected()) {
									if(rdbtn_nm_Spec.isSelected())
									{
										spectroscopyChartPanel = spectroscopyChartReflTrans_NM.getChartPanel();
									}
									else
									{
										spectroscopyChartPanel = spectroscopyChartReflTrans_CM.getChartPanel();
									}
								} else if (rdbtn_Absorbance.isSelected()) {
									if(rdbtn_nm_Spec.isSelected())
									{
										spectroscopyChartPanel = spectroscopyChartAbs_NM.getChartPanel();
									}
									else
									{
										spectroscopyChartPanel = spectroscopyChartAbs_CM.getChartPanel();
									}
								}

								// reset capture after display.
								capture_Displayed_Spectroscopy = false;

								spectroscopyChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));

								spectroscopyChartPanel.setPreferredSize(new Dimension( (int) spectroscopyChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

								this.add(spectroscopyChartPanel, "cell 6 0 10 21,grow");

								spectroscopyChartPanel.repaint();
							} catch (Exception e) {
								logger.error(e.getMessage());
							}
							
							if (data != null) {
								writeRunDataFile(data);
							}
						}
						else
						{
							//Ignore plotting
						}

						data = null;
						xArray = null;
						TArray = null;

						if (UserInterface.stopContinuousRun || !UserInterface.isContinuousRun || !rdbtn_ContRun_Spec.isSelected()) {

							UserInterface.statusLabel.setText("Measurement completed successfully.");
							UserInterface.colorLabel.setForeground(Color.green.darker());
							UserInterface.colorLabel.setBackground(Color.green.darker());

							UserInterface.progressTime = -1;
							UserInterface.progressPar.setValue(100);

						}
					} else {
						UserInterface.colorLabel.setForeground(Color.red);
						UserInterface.colorLabel.setBackground(Color.red);

						UserInterface.progressTime = -1;
						UserInterface.progressPar.setValue(100);

						JOptionPane.showMessageDialog(
								null,
								UserInterface.convertErrorCodesToMessages(resp.getStatus()), "Run NeoSpectra Micro",
								JOptionPane.OK_OPTION);
						btn_Stop_Spec.doClick();

					}
				}

				if (!UserInterface.stopContinuousRun && UserInterface.isContinuousRun && rdbtn_ContRun_Spec.isSelected()) {
					UserInterface.releaseDeviceToken();
					RunSampleSequence();
					try{logger.info("Run_Spec Finished (Without PowerUp) - Scan Time = " + String.valueOf(Double.parseDouble(txt_RunTime_Spec.getText()) * 1000));}catch(Exception d){}
				}
				else
				{
					if(rdbtn_ContRun_Spec.isSelected())
					{
						UserInterface.colorLabel.setForeground(Color.green.darker());
						UserInterface.colorLabel.setBackground(Color.green.darker());
					}


					UserInterface.boardReadyRoutine(true);
					boolean btn_capture_status = btn_Capture_Spec.isEnabled();
					btn_Capture_Spec.setEnabled(true);
					btn_Capture_Spec.doClick();
					btn_Capture_Spec.setEnabled(btn_capture_status);

					// resume checking the status of the device
					UserInterface.releaseDeviceToken();
					UserInterface.checkDeviceStatusThreadStop = false;
				}

				break;
			default:
				// not supported action
				;
			}
			
		}
	}

	/*
	 * !removes last Absorbance chart from GUI, also last Absorbance file
	 */

	public void deleteLastAbsChart() {
		p3AppManagerUtils.createDir(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
		File folder = new File(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
		File[] listOfFiles = folder.listFiles();
		spectroscopyChartAbs_NM.removeSeries("Measurement No. " + (measurementCount_Spectroscopy));
		spectroscopyChartAbs_CM.removeSeries("Measurement No. " + (measurementCount_Spectroscopy));
		for (int i = 0; i < listOfFiles.length; i++) {
			if (listOfFiles[i].isFile()) {
				if (listOfFiles[i].getName().equals("Measurement_Abs_" + (measurementCount_Spectroscopy) + ".Spectrum")) {
					listOfFiles[i].delete();
				}
			}
		}
	}

	/*
	 * !removes last reflection chart from GUI, also last reflection file
	 */
	public void deleteLastReflChart() {
		p3AppManagerUtils.createDir(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
		File folder = new File(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
		File[] listOfFiles = folder.listFiles();
		spectroscopyChartReflTrans_NM.removeSeries("Measurement No. " + (measurementCount_Spectroscopy));
		spectroscopyChartReflTrans_CM.removeSeries("Measurement No. " + (measurementCount_Spectroscopy));
		for (int i = 0; i < listOfFiles.length; i++) {
			if (listOfFiles[i].isFile()) {
				if (listOfFiles[i].getName().equals("Measurement_Refl_" + (measurementCount_Spectroscopy) + ".Spectrum")) {
					listOfFiles[i].delete();
				}
			}
		}
	}

	public void layoutResize(String cell)
	{
		((MigLayout)this.getLayout()).setColumnConstraints("[90.00:90.00:90.00][45.00:45.00:45.00][45.00:45.00:45.00][30.00:30.00:30.00][30.00:30.00:30.00][30.00:30.00:30.00][90.00:90.00:90.00][90.00:90.00:90.00][90.00:90.00:90.00][90.00:90.00:90.00]" + cell + "[][][][][grow,fill]");
	}
	/*
	 * ! writes plot file after Run
	 * 
	 * @param [IN] data: plot sampled points.
	 * 
	 * @param [IN} moduleID: Device ID.
	 * 
	 */
	boolean writeRunDataFile(double[][] data) {
		// spectrum file
		double[] xInverse = UserInterface.switch_NM_CM(data[p3Constants.WAVENUMBER_INDEX]);
		double[] TArray = UserInterface.convertDataToT(data[p3Constants.POWER_SPECTRAL_DENSITY_INDEX]);
		double[] Absorbance = UserInterface.convertRefltoAbs(TArray);
		double transmittanceError = UserInterface.applicationManager.getErrorData();
		for(int i = 0; i < TArray.length; i++)
		{
			TArray[i] = TArray[i] / transmittanceError;
		}

		if(rdbtn_nm_Spec.isSelected())
		{
			// reflectance file
			if (!UserInterface.writeGraphFile(xInverse, TArray, p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_REFL_PATH_TEMPLATE), measurementCount_Spectroscopy), SPECTRUM_FILE_X_AXIS_NM + "\t" + SPECTRUMFILE_Y_AXIS_R_T)) {

				throw new
				p3AppManagerException("Error while writing spectrum file    ",
						p3AppManagerStatus.SPECTRUM_FILE_CREATION_ERROR.getNumVal());

			}
			// absorbance file
			if (!UserInterface.writeGraphFile(xInverse, Absorbance, p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_ABS_PATH_TEMPLATE), measurementCount_Spectroscopy), SPECTRUM_FILE_X_AXIS_NM + "\t" + SPECTRUMFILE_Y_AXIS_A)) {

				throw new
				p3AppManagerException("Error while writing spectrum file    ",
						p3AppManagerStatus.SPECTRUM_FILE_CREATION_ERROR.getNumVal());

			}
		}
		else
		{

			// reflectance file
			if (!UserInterface.writeGraphFile(data[p3Constants.WAVENUMBER_INDEX], TArray, p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_REFL_PATH_TEMPLATE), measurementCount_Spectroscopy), SPECTRUM_FILE_X_AXIS_CM + "\t" + SPECTRUMFILE_Y_AXIS_R_T)) {

				throw new
				p3AppManagerException("Error while writing spectrum file    ",
						p3AppManagerStatus.SPECTRUM_FILE_CREATION_ERROR.getNumVal());

			}
			// absorbance file
			if (!UserInterface.writeGraphFile(data[p3Constants.WAVENUMBER_INDEX], Absorbance, p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_ABS_PATH_TEMPLATE), measurementCount_Spectroscopy), SPECTRUM_FILE_X_AXIS_CM + "\t" + SPECTRUMFILE_Y_AXIS_A)) {

				throw new
				p3AppManagerException("Error while writing spectrum file    ",
						p3AppManagerStatus.SPECTRUM_FILE_CREATION_ERROR.getNumVal());

			}
		}
		xInverse = null;
		TArray = null;
		return true;
	}

	private void RunSampleSequence()
	{
		UserInterface.stopContinuousRun = false;
		UserInterface.isContinuousRun = true;
		// Start RUN
		if (!UserInterface.stopContinuousRun) {

			try
			{
				// stop checking the status of the device
				UserInterface.checkDeviceStatusThreadStop = true;

				try{
					if(UserInterface.boardRestarted)
					{

						//board is restarted, take background again
						JOptionPane.showMessageDialog(null,
								"No valid background measurement. \n Please take background measurement first.",
								"Run NeoSpectra Micro", JOptionPane.OK_OPTION);

						UserInterface.checkDeviceStatusThreadStop = false;
						UserInterface.stopContinuousRun = true;
						UserInterface.isContinuousRun = false;
						return;
					}

					if(!lastOpticalSettingsSelected.equals(cmb_Optical_Settings_Spec.getSelectedItem().toString()))
					{
						//conf. files changed, take background again
						JOptionPane.showMessageDialog(null,
								"You have changed the optical gain settings after your last background measurement. \nPlease retake a new background measurement",
								"Run NeoSpectra Micro", JOptionPane.OK_OPTION);

						UserInterface.checkDeviceStatusThreadStop = false;
						UserInterface.stopContinuousRun = true;
						UserInterface.isContinuousRun = false;
						return;
					}
					if(!lastZeroPaddingSelected.equals(String.valueOf(cmb_ZeroPadding_Spec.getSelectedIndex())))
					{
						//conf. files changed, take background again
						JOptionPane.showMessageDialog(null,
								"You have changed the zero padding settings after your last background measurement. \nPlease retake a new background measurement",
								"Run NeoSpectra Micro", JOptionPane.OK_OPTION);

						UserInterface.checkDeviceStatusThreadStop = false;
						UserInterface.stopContinuousRun = true;
						UserInterface.isContinuousRun = false;
						return;
					}
					if(!lastWindowSelected.equals(String.valueOf(cmb_Apodization_Spec.getSelectedIndex())))
					{
						//conf. files changed, take background again
						JOptionPane.showMessageDialog(null,
								"You have changed the apodization function after your last background measurement. \nPlease retake a new background measurement",
								"Run NeoSpectra Micro", JOptionPane.OK_OPTION);

						UserInterface.checkDeviceStatusThreadStop = false;
						UserInterface.stopContinuousRun = true;
						UserInterface.isContinuousRun = false;
						return;
					}
					
					
					if((chb_EnableLinearInterpolation_Spectroscopy.isSelected() && !lastCommonWavenumberSelected.equals(String.valueOf(cmb_Wavenumber_No_Of_Points_Spec.getSelectedIndex() + 1)))
							|| (!chb_EnableLinearInterpolation_Spectroscopy.isSelected() && !lastCommonWavenumberSelected.equals("0")))
					{
						//conf. files changed, take background again
						JOptionPane.showMessageDialog(null,
								"You have changed the number of data points after your last background measurement. \nPlease retake a new background measurement",
								"Run NeoSpectra Micro", JOptionPane.OK_OPTION);

						UserInterface.checkDeviceStatusThreadStop = false;
						UserInterface.stopContinuousRun = true;
						UserInterface.isContinuousRun = false;
						return;
					}
				}
				catch(Exception ex)
				{
					JOptionPane.showMessageDialog(null,
							"Run failed to start. Module setup failed!. \n Please take another measurement. \nIf the error persists, please contact Si-Ware Systems.",
							"Run NeoSpectra Micro", JOptionPane.OK_OPTION);

					UserInterface.checkDeviceStatusThreadStop = false;
					UserInterface.stopContinuousRun = true;
					UserInterface.isContinuousRun = false;
					return;
				}

				UserInterface.progressTime = Double.parseDouble(txt_RunTime_Spec.getText()) * 1000;
				UserInterface.progressPar.setValue(0);

				
				int commonWavenumberSelection = 0;
				if(chb_EnableLinearInterpolation_Spectroscopy.isSelected())
				{
					commonWavenumberSelection = cmb_Wavenumber_No_Of_Points_Spec.getSelectedIndex() + 1;
				}
				while(UserInterface.acquireDeviceToken() == false);
				p3AppManagerStatus status = UserInterface.applicationManager.runSpec(String.valueOf(Double.parseDouble(txt_RunTime_Spec.getText()) * 1000), "true",
						String.valueOf(cmb_Apodization_Spec.getSelectedIndex()),
						String.valueOf(cmb_ZeroPadding_Spec.getSelectedIndex()), 
						UserInterface.getOpticalGainValue((String)cmb_Optical_Settings_Spec.getSelectedItem()),
						String.valueOf(commonWavenumberSelection), Integer.toString(rdbtn_ContRun_Spec.isSelected()? 1:0));

				if (p3AppManagerStatus.NO_ERROR != status) {
					if (p3AppManagerStatus.DEVICE_BUSY_ERROR != status) {
						// resume checking the status of the device
						UserInterface.checkDeviceStatusThreadStop = false;
					}

					JOptionPane.showMessageDialog(null,
							"Run failed to start: " + UserInterface.convertErrorCodesToMessages(status),
							"Run NeoSpectra Micro", JOptionPane.OK_OPTION);

					UserInterface.stopContinuousRun = true;
					UserInterface.isContinuousRun = false;

					// resume checking the status of the device
					UserInterface.releaseDeviceToken();
				} else {

					UserInterface.boardReadyRoutine(false);

					UserInterface.statusLabel
					.setText("Run started. Please wait...");

					UserInterface.colorLabel.setForeground(Color.yellow);
					UserInterface.colorLabel.setBackground(Color.yellow);

				}
			}
			catch(Exception ex)
			{
				JOptionPane.showMessageDialog(null,
						"Run failed to start. \n Please make sure that the entries in measurement parameters are valid.",
						"Run NeoSpectra Micro", JOptionPane.OK_OPTION);
				
				UserInterface.stopContinuousRun = true;
				UserInterface.isContinuousRun = false;
				UserInterface.boardReadyRoutine(true);
				UserInterface.releaseDeviceToken();
				// resume checking the status of the device
				
				UserInterface.checkDeviceStatusThreadStop = false;

			}
		}
		UserInterface.frmMain.revalidate();
		UserInterface.frmMain.repaint();
	}

	private void LoadCharts(double[][] arraysToPlot)
	{
		// Drawing Charts
		try {
			remove(spectroscopyChartPanel);
			measurementCount_Spectroscopy++;
			// user loaded an absorbance chart
			if (rdbtn_Absorbance.isSelected()) {
				double[] xAxis = UserInterface.switch_NM_CM(arraysToPlot[0]);

				if (measurementCount_Spectroscopy == 1) {
					if(rdbtn_nm_Spec.isSelected())
					{
						spectroscopyChartAbs_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "Absorbance units", arraysToPlot);
						spectroscopyChartAbs_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "Absorbance units",  new double[][] { xAxis, arraysToPlot[1] });
					}
					else
					{
						spectroscopyChartAbs_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "Absorbance units", arraysToPlot);
						spectroscopyChartAbs_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "Absorbance units",  new double[][] { xAxis, arraysToPlot[1] });
					}
				} else {
					if(rdbtn_nm_Spec.isSelected())
					{
						spectroscopyChartAbs_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, arraysToPlot);
						spectroscopyChartAbs_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy,  new double[][] { xAxis, arraysToPlot[1] });
					}
					else
					{
						spectroscopyChartAbs_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, arraysToPlot);
						spectroscopyChartAbs_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy,  new double[][] { xAxis, arraysToPlot[1] });
					}
				}

				if(rdbtn_nm_Spec.isSelected())
				{
					UserInterface.writeGraphFile(arraysToPlot[0], arraysToPlot[1], p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_ABS_PATH_TEMPLATE), measurementCount_Spectroscopy), SPECTRUM_FILE_X_AXIS_NM + "\t" + SPECTRUMFILE_Y_AXIS_A);
					spectroscopyChartPanel = spectroscopyChartAbs_NM.getChartPanel();
				}
				else
				{
					UserInterface.writeGraphFile(arraysToPlot[0], arraysToPlot[1], p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_ABS_PATH_TEMPLATE), measurementCount_Spectroscopy), SPECTRUM_FILE_X_AXIS_CM + "\t" + SPECTRUMFILE_Y_AXIS_A);
					spectroscopyChartPanel = spectroscopyChartAbs_CM.getChartPanel();
				}
				// create the Reflectance chart
				double[] TArray = UserInterface.convertAbstoRefl(arraysToPlot[1]);
				double[] xInverse = arraysToPlot[0];

				if(rdbtn_nm_Spec.isSelected())
				{
					UserInterface.writeGraphFile(xInverse, TArray, p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_REFL_PATH_TEMPLATE), measurementCount_Spectroscopy), SPECTRUM_FILE_X_AXIS_NM + "\t" + SPECTRUMFILE_Y_AXIS_R_T);
				}
				else
				{
					UserInterface.writeGraphFile(xInverse, TArray, p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_REFL_PATH_TEMPLATE), measurementCount_Spectroscopy), SPECTRUM_FILE_X_AXIS_CM + "\t" + SPECTRUMFILE_Y_AXIS_R_T);
				}
				xAxis = UserInterface.switch_NM_CM(arraysToPlot[0]);
				if (measurementCount_Spectroscopy == 1) {
					try
					{
						if(rdbtn_nm_Spec.isSelected())
						{
							spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { xInverse, TArray }, true);
							spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { xAxis, TArray }, true);
						}
						else
						{
							spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { xInverse, TArray }, true);
							spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { xAxis, TArray }, true);
						}
					}
					catch(Exception ex)
					{
						if(rdbtn_nm_Spec.isSelected())
						{
							spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { xInverse, TArray }, true);
							spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { xAxis, TArray }, true);
						}
						else
						{
							spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { xInverse, TArray }, true);
							spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { xAxis, TArray }, true);
						}
					}
				} else {
					if(rdbtn_nm_Spec.isSelected())
					{
						spectroscopyChartReflTrans_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xInverse, TArray }, true);
						spectroscopyChartReflTrans_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, TArray }, true);
					}
					else
					{
						spectroscopyChartReflTrans_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xInverse, TArray }, true);
						spectroscopyChartReflTrans_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, TArray }, true);
					}
				}

				// user loaded a reflectance chart
			} else if (rdbtn_Refl_Trans.isSelected()) {
				double[] xAxis = UserInterface.switch_NM_CM(arraysToPlot[0]);

				if (measurementCount_Spectroscopy == 1) {
					try
					{
						if(rdbtn_nm_Spec.isSelected())
						{
							spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", arraysToPlot, true);
							spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { xAxis, arraysToPlot[1] }, true);
						}
						else
						{
							spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", arraysToPlot, true);
							spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { xAxis, arraysToPlot[1] }, true);
						}
					}
					catch(Exception ex)
					{
						if(rdbtn_nm_Spec.isSelected())
						{
							spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", arraysToPlot, true);
							spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { xAxis, arraysToPlot[1] }, true);
						}
						else
						{
							spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", arraysToPlot, true);
							spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { xAxis, arraysToPlot[1] }, true);
						}
					}
				} else {
					if(rdbtn_nm_Spec.isSelected())
					{
						spectroscopyChartReflTrans_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, arraysToPlot, true);
						spectroscopyChartReflTrans_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, arraysToPlot[1] }, true);
					}
					else
					{
						spectroscopyChartReflTrans_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, arraysToPlot, true);
						spectroscopyChartReflTrans_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, arraysToPlot[1] }, true);
					}
				}
				if(rdbtn_nm_Spec.isSelected())
				{
					spectroscopyChartPanel = spectroscopyChartReflTrans_NM.getChartPanel();
					UserInterface.writeGraphFile(arraysToPlot[0], arraysToPlot[1], p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_REFL_PATH_TEMPLATE), measurementCount_Spectroscopy), SPECTRUM_FILE_X_AXIS_NM + "\t" + SPECTRUMFILE_Y_AXIS_R_T);
				}
				else
				{
					spectroscopyChartPanel = spectroscopyChartReflTrans_CM.getChartPanel();
					UserInterface.writeGraphFile(arraysToPlot[0], arraysToPlot[1], p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_REFL_PATH_TEMPLATE), measurementCount_Spectroscopy), SPECTRUM_FILE_X_AXIS_CM + "\t" + SPECTRUMFILE_Y_AXIS_R_T);
				}
				// convert loaded Reflectance to Absorbance and fill it's plot.
				double[] Absorbance = UserInterface.convertRefltoAbs(arraysToPlot[1]);
				double[] xInverse = arraysToPlot[0];

				if(rdbtn_nm_Spec.isSelected())
				{
					UserInterface.writeGraphFile(xInverse, Absorbance, p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_ABS_PATH_TEMPLATE), measurementCount_Spectroscopy), SPECTRUM_FILE_X_AXIS_NM + "\t" + SPECTRUMFILE_Y_AXIS_A);
				}
				else
				{
					UserInterface.writeGraphFile(xInverse, Absorbance, p3AppManagerUtils.formatString(p3Constants.getPath(SPECTRUM_ABS_PATH_TEMPLATE), measurementCount_Spectroscopy), SPECTRUM_FILE_X_AXIS_CM + "\t" + SPECTRUMFILE_Y_AXIS_A);
				}

				xAxis = UserInterface.switch_NM_CM(arraysToPlot[0]);

				if (measurementCount_Spectroscopy == 1) {
					if(rdbtn_nm_Spec.isSelected())
					{
						spectroscopyChartAbs_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "Absorbance", new double[][] { xInverse, Absorbance }, true);
						spectroscopyChartAbs_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "Absorbance", new double[][] { xAxis, Absorbance }, true);
					}
					else
					{
						spectroscopyChartAbs_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "Absorbance", new double[][] { xInverse, Absorbance }, true);
						spectroscopyChartAbs_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "Absorbance", new double[][] { xAxis, Absorbance }, true);
					}
				} else {
					if(rdbtn_nm_Spec.isSelected())
					{
						spectroscopyChartAbs_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xInverse, Absorbance }, true);
						spectroscopyChartAbs_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, Absorbance }, true);
					}
					else
					{
						spectroscopyChartAbs_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xInverse, Absorbance }, true);
						spectroscopyChartAbs_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, Absorbance }, true);
					}
				}
			}
			// capture the loaded plot only.
			boolean btn_capture_status = btn_Capture_Spec.isEnabled();
			btn_Capture_Spec.setEnabled(true);
			btn_Capture_Spec.doClick();
			btn_Capture_Spec.setEnabled(btn_capture_status);

			spectroscopyChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));
			spectroscopyChartPanel.repaint();

			spectroscopyChartPanel.setPreferredSize(new Dimension( (int) spectroscopyChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

			add(spectroscopyChartPanel, "cell 6 0 10 21,grow");
			repaint();
			UserInterface.frmMain.revalidate();
			UserInterface.frmMain.repaint();

		} catch (Exception ex) {
			JOptionPane.showMessageDialog(null,
					"Failed to save measurement data files in the temporary directory: "
							+ ex.getMessage(), "Saving files",
							JOptionPane.OK_OPTION);
		}
	}

	public void changeOptionRoutine() {

		boolean absState = rdbtn_Absorbance.isSelected();

		File directory = new File(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
		if(!directory.exists())
		{
			p3AppManagerUtils.createDir(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
		}

		// get all the files from the temp plots directory
		File[] fList = directory.listFiles();
		measurementCount_Spectroscopy = 0;
		if(fList.length == 0)
		{
			spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "", "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { new double[] { SPECTRUM_X_NM_MIN }, new double[] { SPECTRUM_Y_REF_MIN } }, SPECTRUM_X_NM_MIN, SPECTRUM_X_NM_MAX, SPECTRUM_Y_REF_MIN, SPECTRUM_Y_REF_MAX);
			spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "", "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { new double[] { SPECTRUM_X_CM_MIN }, new double[] { SPECTRUM_Y_REF_MIN } }, SPECTRUM_X_CM_MIN, SPECTRUM_X_CM_MAX, SPECTRUM_Y_REF_MIN, SPECTRUM_Y_REF_MAX);

			if(rdbtn_Refl_Trans.isSelected())
			{
				remove(spectroscopyChartPanel);

				if(rdbtn_nm_Spec.isSelected())
				{
					spectroscopyChartPanel = spectroscopyChartReflTrans_NM.getChartPanel();
				}
				else
				{
					spectroscopyChartPanel = spectroscopyChartReflTrans_CM.getChartPanel();
				}
				spectroscopyChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));
				spectroscopyChartPanel.getChart().removeLegend();

				spectroscopyChartPanel.setPreferredSize(new Dimension( (int) spectroscopyChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

				add(spectroscopyChartPanel, "cell 6 0 10 21,grow");
				repaint();
				UserInterface.frmMain.revalidate();
				UserInterface.frmMain.repaint();
			}
		}
		else
		{
			for (File file : fList) {
				if (!file.isDirectory() && file.getName().endsWith(".Spectrum") && file.getName().contains("Abs")) {

					String filePath = file.getAbsolutePath();

					double[][] arraysToPlot = UserInterface
							.loadGraphDataFromFile(filePath);

					double[] xAxis = UserInterface.switch_NM_CM(arraysToPlot[0]);

					// Drawing Charts
					try {
						remove(spectroscopyChartPanel);
						measurementCount_Spectroscopy++;
						// user loaded an absorbance chart
						if (rdbtn_Absorbance.isSelected()) {
							if (measurementCount_Spectroscopy == 1) {
								if(rdbtn_nm_Spec.isSelected())
								{
									spectroscopyChartAbs_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "Absorbance units", arraysToPlot, true);
									spectroscopyChartAbs_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "Absorbance units",  new double[][] { xAxis, arraysToPlot[1] }, true);
								}
								else
								{
									spectroscopyChartAbs_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "Absorbance units", arraysToPlot, true);
									spectroscopyChartAbs_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "Absorbance units",  new double[][] { xAxis, arraysToPlot[1] }, true);
								}
							} else {
								if(rdbtn_nm_Spec.isSelected())
								{
									spectroscopyChartAbs_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, arraysToPlot, true);
									spectroscopyChartAbs_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, arraysToPlot[1] }, true);
								}
								else
								{
									spectroscopyChartAbs_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, arraysToPlot, true);
									spectroscopyChartAbs_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, arraysToPlot[1] }, true);
								}
							}

							if(rdbtn_nm_Spec.isSelected())
							{
								spectroscopyChartPanel = spectroscopyChartAbs_NM.getChartPanel();
							}
							else
							{
								spectroscopyChartPanel = spectroscopyChartAbs_CM.getChartPanel();
							}

							// create the Reflectance chart
							double[] TArray = UserInterface.convertAbstoRefl(arraysToPlot[1]);
							double[] xInverse = arraysToPlot[0];

							if (measurementCount_Spectroscopy == 1) {
								if(rdbtn_nm_Spec.isSelected())
								{
									spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { xInverse, TArray }, true);
									spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { xAxis, TArray }, true);
								}
								else
								{
									spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { xInverse, TArray }, true);
									spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { xAxis, TArray }, true);
								}
							} else {
								if(rdbtn_nm_Spec.isSelected())
								{
									spectroscopyChartReflTrans_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xInverse, TArray }, true);
									spectroscopyChartReflTrans_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, TArray }, true);
								}
								else
								{
									spectroscopyChartReflTrans_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xInverse, TArray }, true);
									spectroscopyChartReflTrans_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, TArray }, true);
								}
							}

							// user loaded a reflectance chart
						} else if (rdbtn_Refl_Trans.isSelected()) {
							if (measurementCount_Spectroscopy == 1) {
								if(rdbtn_nm_Spec.isSelected())
								{
									spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", arraysToPlot, true);
									spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", new double[][] { xAxis, arraysToPlot[1] }, true);
								}
								else
								{
									spectroscopyChartReflTrans_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "%Reflectance or Transmittance", arraysToPlot, true);
									spectroscopyChartReflTrans_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "%Reflectance or Transmittance", new double[][] { xAxis, arraysToPlot[1] }, true);
								}
							} else {
								if(rdbtn_nm_Spec.isSelected())
								{
									spectroscopyChartReflTrans_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, arraysToPlot, true);
									spectroscopyChartReflTrans_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, arraysToPlot[1] }, true);
								}
								else
								{
									spectroscopyChartReflTrans_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, arraysToPlot, true);
									spectroscopyChartReflTrans_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, arraysToPlot[1] }, true);
								}
							}

							if(rdbtn_nm_Spec.isSelected())
							{
								spectroscopyChartPanel = spectroscopyChartReflTrans_NM.getChartPanel();
							}
							else
							{
								spectroscopyChartPanel = spectroscopyChartReflTrans_CM.getChartPanel();
							}

							// convert loaded Reflectance to Absorbance and fill it's plot.
							double[] Absorbance = UserInterface.convertRefltoAbs(arraysToPlot[1]);
							double[] xInverse = arraysToPlot[0];

							if (measurementCount_Spectroscopy == 1) {
								if(rdbtn_nm_Spec.isSelected())
								{
									spectroscopyChartAbs_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "Absorbance units", new double[][] { xInverse, Absorbance }, true);
									spectroscopyChartAbs_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "Absorbance units", new double[][] { xAxis, Absorbance }, true);
								}
								else
								{
									spectroscopyChartAbs_CM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavenumber (cm -\u00B9)", "Absorbance units", new double[][] { xInverse, Absorbance }, true);
									spectroscopyChartAbs_NM = new XYLineChart("Spectrum", "Measurement No. " + measurementCount_Spectroscopy, "Wavelength (nm)", "Absorbance units", new double[][] { xAxis, Absorbance }, true);
								}
							} else {
								if(rdbtn_nm_Spec.isSelected())
								{
									spectroscopyChartAbs_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xInverse, Absorbance }, true);
									spectroscopyChartAbs_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, Absorbance }, true);
								}
								else
								{
									spectroscopyChartAbs_CM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xInverse, Absorbance }, true);
									spectroscopyChartAbs_NM.addSeries("Measurement No. " + measurementCount_Spectroscopy, new double[][] { xAxis, Absorbance }, true);
								}
							}
						}

					} catch (Exception ex) {
						JOptionPane.showMessageDialog(null,
								"Failed to save measurement data files in the temporary directory: "
										+ ex.getMessage(), "Saving files",
										JOptionPane.OK_OPTION);
					}

				}
			}
		}
		spectroscopyChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));
		spectroscopyChartPanel.repaint();

		spectroscopyChartPanel.setPreferredSize(new Dimension( (int) spectroscopyChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

		add(spectroscopyChartPanel, "cell 6 0 10 21,grow");
		repaint();
		UserInterface.frmMain.revalidate();
		UserInterface.frmMain.repaint();

		//after loading the plot will be on absorbance by default
		//so if the original state was refl/trans, we need to convert it back
		if(!absState)
		{
			rdbtn_Refl_Trans.doClick();
		}
	}

	public static void saveGraphsRoutine()
	{
		p3AppManagerUtils.createDir(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
		File folder = new File(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
		File[] listOfFiles = folder.listFiles();

		String[] finalFileNames = new String[listOfFiles.length];
		for(int i = 0; i < finalFileNames.length; i++)
		{
			finalFileNames[i] = UserInterface.fileNameToSave.equals("!Default!") ? listOfFiles[i].getName() : UserInterface.fileNameToSave + listOfFiles[i].getName().substring(listOfFiles[i].getName().indexOf("_"));
		}

		JFileChooser saveSpecFile;
		if(UserInterface.defaultDirectorySaveLoad.equals("") || !new File(UserInterface.defaultDirectorySaveLoad).exists())
		{
			saveSpecFile = new JFileChooser();
		}
		else
		{
			saveSpecFile = new JFileChooser(UserInterface.defaultDirectorySaveLoad);
		}
		saveSpecFile.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);

		saveSpecFile.setApproveButtonText("Save");
		saveSpecFile.setDialogTitle("Save");

		if (finalFileNames.length != 0) {
			if (saveSpecFile.showOpenDialog(null) == JFileChooser.APPROVE_OPTION) {

				BufferedReader reader = null;

				p3AppManagerUtils.createDir(saveSpecFile.getSelectedFile().getAbsolutePath());
				int overwriteRemember = -1;

				for (int i = 0; i < finalFileNames.length; i++) {
					if (listOfFiles[i].isFile()) {
						if (listOfFiles[i].getAbsolutePath().endsWith(".Spectrum")) {
							try {
								//overwriting will happen
								if(new File(saveSpecFile.getSelectedFile().getAbsolutePath() + File.separatorChar + finalFileNames[i]).exists())
								{
									if(overwriteRemember == -1)
									{
										int result = JOptionPane.showConfirmDialog(
												null,
												"The chosen directory has plot(s) with the same name, overwrite all?",
												"Overwrite",
												JOptionPane.YES_NO_OPTION);
										switch (result) {
										case JOptionPane.YES_OPTION:
										{
											overwriteRemember = 1;
											if (rdbtn_Refl_Trans.isSelected()) {
												reader = new BufferedReader(new FileReader(listOfFiles[i].getAbsolutePath()));

												// Read units line check whether Refl/Abs
												String line = reader.readLine();
												if (line.contains("y_Axis:%Reflectance") || line.contains("y_Axis:%Transmittance")) {
													Files.copy(listOfFiles[i].toPath(), (new File(saveSpecFile.getSelectedFile().getAbsolutePath() + File.separatorChar + finalFileNames[i])).toPath(), StandardCopyOption.REPLACE_EXISTING);
												}
												else
												{
													Files.delete((new File(saveSpecFile.getSelectedFile().getAbsolutePath() + File.separatorChar + finalFileNames[i])).toPath());
												}

												reader.close();
											} else {
												reader = new BufferedReader(new FileReader(listOfFiles[i].getAbsolutePath()));

												// Read units line check whether Refl/Abs
												String line = reader.readLine();
												if (line.contains("y_Axis:Absorbance")) {
													Files.copy(listOfFiles[i].toPath(), (new File(saveSpecFile.getSelectedFile().getAbsolutePath() + File.separatorChar + finalFileNames[i])).toPath(), StandardCopyOption.REPLACE_EXISTING);
												}
												else
												{
													Files.delete((new File(saveSpecFile.getSelectedFile().getAbsolutePath() + File.separatorChar + finalFileNames[i])).toPath());
												}
												reader.close();
											}
											break;
										}
										case JOptionPane.CLOSED_OPTION:
										{
											overwriteRemember = 0;
											break;
										}
										case JOptionPane.NO_OPTION:
										{
											overwriteRemember = 0;
											break;
										}
										}
									}
									else if (overwriteRemember == 1)
									{
										if (rdbtn_Refl_Trans.isSelected()) {
											reader = new BufferedReader(new FileReader(listOfFiles[i].getAbsolutePath()));

											// Read units line check whether Refl/Abs
											String line = reader.readLine();
											if (line.contains("y_Axis:%Reflectance") || line.contains("y_Axis:%Transmittance")) {
												Files.copy(listOfFiles[i].toPath(), (new File(saveSpecFile.getSelectedFile().getAbsolutePath() + File.separatorChar + finalFileNames[i])).toPath(), StandardCopyOption.REPLACE_EXISTING);
											}
											reader.close();
										} else {
											reader = new BufferedReader(new FileReader(listOfFiles[i].getAbsolutePath()));

											// Read units line check whether Refl/Abs
											String line = reader.readLine();
											if (line.contains("y_Axis:Absorbance")) {
												Files.copy(listOfFiles[i].toPath(), (new File(saveSpecFile.getSelectedFile().getAbsolutePath() + File.separatorChar + finalFileNames[i])).toPath(), StandardCopyOption.REPLACE_EXISTING);
											}

											reader.close();
										}
									}
								}
								//no overwriting will happen
								else
								{
									if (rdbtn_Refl_Trans.isSelected()) {
										reader = new BufferedReader(new FileReader(listOfFiles[i].getAbsolutePath()));

										// Read units line check whether Refl/Abs
										String line = reader.readLine();
										if (line.contains("y_Axis:%Reflectance") || line.contains("y_Axis:%Transmittance")) {
											Files.copy(listOfFiles[i].toPath(), (new File(saveSpecFile.getSelectedFile().getAbsolutePath() + File.separatorChar + finalFileNames[i])).toPath(), StandardCopyOption.REPLACE_EXISTING);
										}
										reader.close();
									} else {
										reader = new BufferedReader(new FileReader(listOfFiles[i].getAbsolutePath()));

										// Read units line check whether Refl/Abs
										String line = reader.readLine();
										if (line.contains("y_Axis:Absorbance")) {
											Files.copy(listOfFiles[i].toPath(), (new File(saveSpecFile.getSelectedFile().getAbsolutePath() + File.separatorChar + finalFileNames[i])).toPath(), StandardCopyOption.REPLACE_EXISTING);
										}

										reader.close();
									}
								}

							} catch (Exception ex) {
								JOptionPane.showMessageDialog(null,
										"Failed to save files to the selected destination: "
												+ ex.getMessage(),
												"Saving files",
												JOptionPane.OK_OPTION);
								return;
							}
						}
					}
				}
				UserInterface.defaultDirectorySaveLoad = saveSpecFile.getSelectedFile().getAbsolutePath();
			}
		}
	}
}
