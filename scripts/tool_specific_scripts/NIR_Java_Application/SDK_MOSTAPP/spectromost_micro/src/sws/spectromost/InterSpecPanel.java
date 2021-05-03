package sws.spectromost;

import java.awt.event.ActionEvent;

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
import javax.swing.SwingConstants;

import java.awt.event.ActionListener;

import javax.swing.JButton;

import java.io.File;
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

import javax.swing.JPanel;
import javax.swing.event.PopupMenuEvent;
import javax.swing.event.PopupMenuListener;
import javax.swing.filechooser.FileNameExtensionFilter;
import javax.swing.JSeparator;

import java.awt.Font;
import java.awt.Color;

import javax.swing.DefaultComboBoxModel;
import javax.swing.border.LineBorder;

import java.awt.Label;

import javax.swing.JRadioButton;

import org.apache.log4j.Logger;

@SuppressWarnings("serial")
public class InterSpecPanel extends JPanel {

	private static Logger logger = Logger.getLogger(InterSpecPanel.class);
	/*
	 * ! Interferogram & Spectrum tab
	 */
	public static final String PSD_PATH_TEMPLATE = p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH) + File.separatorChar + "Measurement_{0}.InterPSD";
	public static final String PSD_FILE_X_AXIS_CM = "x_Axis:Wavenumber (cm-1)";
	public static final String PSD_FILE_X_AXIS_NM = "x_Axis:Wavelength (nm)";
	public static final String PSD_FILE_Y_AXIS_N = "y_Axis:Normalized PSD (a.u.)";
	public static final String PSD_FILE_Y_AXIS = "y_Axis:PSD (a.u.)";

	/*
	 * ! boolean to check whether a capture button was pressed
	 */
	public static boolean capture_Displayed_InterSpec = false;

	/*
	 * ! boolean to guard the clearing plots method
	 */
	private static boolean clearingGraphsInProgress = false;

	/*
	 * ! Auto Save Path for Interferogram & spectroscopy tab
	 */
	private String AutoSavePath_Inter_Spec = "";

	/*
	 * ! The last selected resolution
	 */
	public static String lastResolutionSelected = "";

	/*
	 * ! The last selected opticalGainSettings
	 */
	public static String lastOpticalSettingsSelected = "";

	/*
	 * !GUI fields
	 */
	public static JRadioButton rdbtn_nm_Inter_Spec = new JRadioButton("nm");
	public static JRadioButton rdbtn_cm_Inter_Spec = new JRadioButton("cm\u207B\u00B9");
	static Label lbl_ModuleID_Inter_Spec;
	static XYLineChart spectrumChart_NM = null;
	static XYLineChart spectrumChart_CM = null;
	static ChartPanel spectrumChartPanel = null;
	public static final JCheckBox chb_AutoSave_Inter_Spec = new JCheckBox("Auto-save");
	public static final JCheckBox chb_EnableLinearInterpolation_Inter_Spec = new JCheckBox("Enable linear interpolaion");
	static JButton btn_Run_Inter_Spec;
	static JButton btn_Capture_Interfero;
	static JButton btn_ClearGraphs_Inter_Spec;
	static JButton btn_LoadGraphs_Inter_Spec;
	static JButton btn_SaveGraphs_Inter_Spec;
	static JButton btn_Stop_Inter_Spec;
	static JButton btn_GainAdjustmentPanel_Inter_Spec;
	static JButton btn_WavelengthWavenumberCalibration_Inter_Spec;
	static JButton btn_WriteSettings_Inter_Spec;
	static JButton btn_RestoreDefault_Inter_Spec;
	static JButton btn_DeleteOpticalOption_Inter_Spec;
	static JComboBox<String> cmb_Optical_Settings_Inter_Spec;
	static JFormattedTextField txt_RunTime_Inter_Spec;
	static JComboBox<String> cmb_Wavenumber_No_Of_Points_Inter_Spec;
	static JComboBox<String> cmb_Apodization_Inter_Spec;
	static JComboBox<String> cmb_ZeroPadding_Inter_Spec;
	static Label lbl_advancedSettings_Inter_Spec;
	static Label lbl_FFT_Settings_Inter_Spec;
	static JSeparator separatorV1_Inter_Spec;
	static Label lbl_Apodization_Inter_Spec;
	static Label lbl_ZeroPadding_Inter_Spec;
	private int measurementCount_Spectrum = 0;

	
	/*
	 * ! PSD's axis default values
	 */
	public static final double PSD_X_NM_MIN = 1285;
	public static final double PSD_X_NM_MAX = 2615;
	public static final double PSD_X_CM_MIN = 3700;
	public static final double PSD_X_CM_MAX = 7795;
	public static final double PSD_Y_MIN = 0;
	public static final double PSD_Y_MAX = 1.2;
	private Label lbl_SpectrometerInfo_Inter_Spec;
	public static JRadioButton rdbtn_SingleRun_Inter_Spec;
	public static JRadioButton rdbtn_ContRun_Inter_Spec;
	private Label lblDataDisplay_Inter_Spec;
	private Label lblXaxis_Inter_Spec;
	private boolean genuineAction = false;

	public InterSpecPanel() {
		super();
		this.initialize();
	}

	public void initialize() {

		this.setFont(new Font("Dialog", Font.PLAIN, 12));
		this.setBackground(new Color(176, 196, 222));
		this.setLayout(new MigLayout("", "[90.00:90.00:90.00][45.00:45.00:45.00][45.00:45.00:45.00][30.00:30.00:30.00][30.00:30.00:30.00][30.00:30.00:30.00][90.00:90.00:90.00][90.00:90.00:90.00][90.00:90.00:90.00][90.00:90.00:90.00][grow,fill][][][][][grow,fill]", "[][][][][][][][][][][][][][][][][][25.00:25.00:25.00][20:n][][][][][][][][][][][][grow,fill]"));

		lbl_SpectrometerInfo_Inter_Spec = new Label("NeoSpectra Micro kit Info");
		lbl_SpectrometerInfo_Inter_Spec.setFont(new Font("Dialog", Font.BOLD, 12));
		add(lbl_SpectrometerInfo_Inter_Spec, "cell 0 0 6 1");

		Label lbl_SpectrometerID_Inter_Spec = new Label("Kit ID");
		lbl_SpectrometerID_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		this.add(lbl_SpectrometerID_Inter_Spec, "cell 0 1 2 1,growx");

		lbl_ModuleID_Inter_Spec = new Label(UserInterface.moduleID);
		lbl_ModuleID_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		this.add(lbl_ModuleID_Inter_Spec, "cell 2 1 2 1,alignx right");

		JSeparator separator1_Inter_Spec = new JSeparator();
		add(separator1_Inter_Spec, "cell 0 2 6 1,growx");

		Label lbl_MeasurementParams_Inter_Spec = new Label("Measurement Parameters");
		lbl_MeasurementParams_Inter_Spec.setFont(new Font("Dialog", Font.BOLD, 12));
		this.add(lbl_MeasurementParams_Inter_Spec, "cell 0 3 6 1,growx");

		Label lbl_RunTime_Inter_Spec = new Label("Scan Time");
		lbl_RunTime_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		this.add(lbl_RunTime_Inter_Spec, "cell 0 4,growx");

		txt_RunTime_Inter_Spec = new JFormattedTextField();
		txt_RunTime_Inter_Spec.setEditable(true);
		txt_RunTime_Inter_Spec.setFont(new Font("Dialog", Font.BOLD, 10));
		txt_RunTime_Inter_Spec.setText("2");
		txt_RunTime_Inter_Spec.setMinimumSize(new Dimension(p3Constants.MAX_WIDTH_OF_FIELD, txt_RunTime_Inter_Spec.getPreferredSize().height));
		txt_RunTime_Inter_Spec.setMaximumSize(new Dimension(p3Constants.MAX_WIDTH_OF_FIELD, txt_RunTime_Inter_Spec.getPreferredSize().height));
		txt_RunTime_Inter_Spec.setPreferredSize(new Dimension(p3Constants.MAX_WIDTH_OF_FIELD, txt_RunTime_Inter_Spec.getPreferredSize().height));
		this.add(txt_RunTime_Inter_Spec, "cell 2 4 3 1,alignx left");

		Label lbl_Sec_Inter_Spec = new Label("s");
		lbl_Sec_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		this.add(lbl_Sec_Inter_Spec, "cell 5 4,growx");

		Label lbl_Optical_Gain = new Label("Optical Gain Settings");
		lbl_Optical_Gain.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(lbl_Optical_Gain, "cell 0 5,growx");

		cmb_Optical_Settings_Inter_Spec = new JComboBox<String>();
		cmb_Optical_Settings_Inter_Spec.setEnabled(false);
		cmb_Optical_Settings_Inter_Spec.setEditable(false);
		cmb_Optical_Settings_Inter_Spec.setFont(new Font("Dialog", Font.BOLD, 10));
		cmb_Optical_Settings_Inter_Spec.setMaximumSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_Optical_Settings_Inter_Spec.getPreferredSize().height));
		this.add(cmb_Optical_Settings_Inter_Spec, "cell 2 5 4 1,growx");

		cmb_Optical_Settings_Inter_Spec.addPopupMenuListener(new PopupMenuListener() {
			public void popupMenuCanceled(PopupMenuEvent e) {
			}

			public void popupMenuWillBecomeInvisible(PopupMenuEvent e) {
				SpectroscopyPanel.cmb_Optical_Settings_Spec.setSelectedIndex(cmb_Optical_Settings_Inter_Spec.getSelectedIndex());
			}

			public void popupMenuWillBecomeVisible(PopupMenuEvent e) {
				UserInterface.displaySpecOpticalSettings();
				UserInterface.displayInterSpecOpticalSettings();
			}
		});

		btn_DeleteOpticalOption_Inter_Spec = new JButton("");
		btn_DeleteOpticalOption_Inter_Spec.setEnabled(false);
		btn_DeleteOpticalOption_Inter_Spec.setVisible(true);
		btn_DeleteOpticalOption_Inter_Spec.setToolTipText("Delete the selected optical gain setting");
		btn_DeleteOpticalOption_Inter_Spec.setMaximumSize(new Dimension(30, cmb_Optical_Settings_Inter_Spec.getPreferredSize().height));
		btn_DeleteOpticalOption_Inter_Spec.setMinimumSize(new Dimension(30, cmb_Optical_Settings_Inter_Spec.getPreferredSize().height));
		btn_DeleteOpticalOption_Inter_Spec.setPreferredSize(new Dimension(30, cmb_Optical_Settings_Inter_Spec.getPreferredSize().height));
		btn_DeleteOpticalOption_Inter_Spec.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {

				if(cmb_Optical_Settings_Inter_Spec.getSelectedItem().toString().equals("Default"))
				{
					JOptionPane.showMessageDialog(null,
							"Default Option cannot be deleted!",
							"Delete optical gain", JOptionPane.OK_OPTION);
					return;
				}

				if(JOptionPane.showConfirmDialog(null, "Are you sure you want to delete " + cmb_Optical_Settings_Inter_Spec.getSelectedItem().toString() + " ?", 
						"Delete optical gain", JOptionPane.YES_NO_OPTION) == JOptionPane.NO_OPTION)
				{
					return;
				}

				UserInterface.deleteOpticalSetting(cmb_Optical_Settings_Inter_Spec.getSelectedItem().toString());
				UserInterface.displaySpecOpticalSettings();
				UserInterface.displayInterSpecOpticalSettings();
			}
		});
		add(btn_DeleteOpticalOption_Inter_Spec, "cell 5 5,grow");

		Label lbl_RunMode_Inter_Spec = new Label("Run Mode");
		lbl_RunMode_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(lbl_RunMode_Inter_Spec, "cell 0 6,growx");

		ButtonGroup bgRunMode_Inter_Spec = new ButtonGroup();
		rdbtn_SingleRun_Inter_Spec = new JRadioButton("Single");
		rdbtn_SingleRun_Inter_Spec.setEnabled(false);
		rdbtn_SingleRun_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		rdbtn_SingleRun_Inter_Spec.setSelected(true);
		rdbtn_SingleRun_Inter_Spec.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				btn_Capture_Interfero.setEnabled(false);
			}
		});
		add(rdbtn_SingleRun_Inter_Spec, "cell 1 6 2 1,growx");
		bgRunMode_Inter_Spec.add(rdbtn_SingleRun_Inter_Spec);


		btn_Run_Inter_Spec = new JButton("");
		btn_Run_Inter_Spec.setEnabled(false);
		btn_Run_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_Run_Inter_Spec.setText("Run");
		btn_Run_Inter_Spec.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_Run_Inter_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_Run_Inter_Spec.setToolTipText("Run");

		btn_Run_Inter_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				RunSequence();
				try{logger.info("Run_Inter_Spec Finished (With PowerUp) - Scan Time = " + String.valueOf(Double.parseDouble(txt_RunTime_Inter_Spec.getText()) * 1000));}catch(Exception d){}
			}
		});

		rdbtn_ContRun_Inter_Spec = new JRadioButton("Cont.");
		rdbtn_ContRun_Inter_Spec.setEnabled(false);
		rdbtn_ContRun_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		rdbtn_ContRun_Inter_Spec.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				btn_Capture_Interfero.setEnabled(true);
			}
		});
		add(rdbtn_ContRun_Inter_Spec, "cell 3 6 3 1,growx");
		bgRunMode_Inter_Spec.add(rdbtn_ContRun_Inter_Spec);

		this.add(btn_Run_Inter_Spec, "cell 1 7 2 1,growx");

		btn_Stop_Inter_Spec = new JButton("Stop");
		btn_Stop_Inter_Spec.setEnabled(false);
		btn_Stop_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_Stop_Inter_Spec.setText("Stop");
		btn_Stop_Inter_Spec.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_Stop_Inter_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_Stop_Inter_Spec.setToolTipText("Stop");

		btn_Stop_Inter_Spec.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				UserInterface.stopContinuousRun = true;
				UserInterface.isContinuousRun = false;

				UserInterface.statusLabel.setText("Stopping NeoSpectra Micro kit. Please wait...");
			}
		});
		this.add(btn_Stop_Inter_Spec, "cell 3 7 3 1,growx");

		JSeparator separator2_Inter_Spec = new JSeparator();
		this.add(separator2_Inter_Spec, "cell 0 8 6 1,growx");

		btn_SaveGraphs_Inter_Spec = new JButton("");
		btn_SaveGraphs_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_SaveGraphs_Inter_Spec.setText("Save");
		btn_SaveGraphs_Inter_Spec.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_SaveGraphs_Inter_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_SaveGraphs_Inter_Spec.setToolTipText("Save Plots");
		btn_SaveGraphs_Inter_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {

				UserInterface.stopEnablingButtons = true;
				UserInterface.boardReadyRoutine(false);
				btn_ClearGraphs_Inter_Spec.setEnabled(false);

				String [] graphs = spectrumChart_NM.getSeriesKeys();
				int graph_count = graphs.length;
				for(String graph : graphs)
				{
					double[][] data_NM = spectrumChart_NM.getGraphData(graph);
					double[][] data_CM = spectrumChart_CM.getGraphData(graph);

					if(rdbtn_nm_Inter_Spec.isSelected())
					{
						UserInterface.writeGraphFile(data_NM[0], data_NM[1], 
								p3AppManagerUtils.formatString(p3Constants.getPath(PSD_PATH_TEMPLATE), graph_count), PSD_FILE_X_AXIS_NM + "\t" + PSD_FILE_Y_AXIS); 	
					}
					else
					{
						UserInterface.writeGraphFile(data_CM[0], data_CM[1], 
								p3AppManagerUtils.formatString(p3Constants.getPath(PSD_PATH_TEMPLATE), graph_count), PSD_FILE_X_AXIS_CM + "\t" + PSD_FILE_Y_AXIS); 
					}
					graph_count--;
				}

				SaveGraphsPanel savePanel = new SaveGraphsPanel(p3Constants.InterSpecPrefix);
				savePanel.frmSaveGraphs.setVisible(true);
			}
		});

		this.add(btn_SaveGraphs_Inter_Spec, "cell 0 10,growx");

		btn_LoadGraphs_Inter_Spec = new JButton("");
		btn_LoadGraphs_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_LoadGraphs_Inter_Spec.setText("Load");
		btn_LoadGraphs_Inter_Spec.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_LoadGraphs_Inter_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_LoadGraphs_Inter_Spec.setToolTipText("Load Plot");

		btn_LoadGraphs_Inter_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {

				// remove any previous uncaptured plots
				if ((!capture_Displayed_InterSpec) && (measurementCount_Spectrum != 0)) {
					measurementCount_Spectrum--;
				}
				capture_Displayed_InterSpec = false;

				JFileChooser openInterFile;
				if(UserInterface.defaultDirectorySaveLoad.equals("") || !new File(UserInterface.defaultDirectorySaveLoad).exists())
				{
					openInterFile = new JFileChooser();
				}
				else
				{
					openInterFile = new JFileChooser(UserInterface.defaultDirectorySaveLoad);
				}
				openInterFile.setMultiSelectionEnabled(true);
				openInterFile.setFileFilter(new FileNameExtensionFilter("Interferogram or PSD plots (*.Interferogram , *.InterPSD)", "Interferogram", "InterPSD"));
				if (openInterFile.showOpenDialog(null) == JFileChooser.APPROVE_OPTION) {
					for(File loadedFile: openInterFile.getSelectedFiles())
					{
						LoadCharts(loadedFile);
					}


					UserInterface.defaultDirectorySaveLoad = openInterFile.getSelectedFile().getParentFile().getAbsolutePath();
				}
			}
		});
		this.add(btn_LoadGraphs_Inter_Spec, "cell 1 10 2 1,growx");

		btn_Capture_Interfero = new JButton("Capture");
		btn_Capture_Interfero.setEnabled(false);
		btn_Capture_Interfero.setToolTipText("Capture Plot");
		btn_Capture_Interfero.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_Capture_Interfero.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_Capture_Interfero.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_Capture_Interfero.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent e) {
				if (!capture_Displayed_InterSpec) {
					capture_Displayed_InterSpec = true;
					if ((!AutoSavePath_Inter_Spec.equals("")) && (capture_Displayed_InterSpec)) {
						try {
							p3AppManagerUtils.createDir(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
							File folder = new File(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
							File[] listOfFiles = folder.listFiles();
							if (listOfFiles.length != 0) {

								for (int i = 0; i < listOfFiles.length; i++) {
									if (listOfFiles[i].isFile()) {
										if (listOfFiles[i].getAbsolutePath().endsWith(".Interferogram") || listOfFiles[i].getAbsolutePath().endsWith(".InterPSD")) {
											try {
												p3AppManagerUtils.createDir(AutoSavePath_Inter_Spec);
												Files.copy(listOfFiles[i].toPath(), (new File(AutoSavePath_Inter_Spec + File.separatorChar + listOfFiles[i].getName())).toPath(), StandardCopyOption.REPLACE_EXISTING);

											} catch (Exception ex) {
												JOptionPane.showMessageDialog(
														null,
														"Failed to save files to the selected destination: "
																+ ex.getMessage(),
																"Saving files",
																JOptionPane.OK_OPTION);
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

		this.add(btn_Capture_Interfero, "cell 1 11 2 1,growx");

		JSeparator separator3_Inter_Spec = new JSeparator();
		this.add(separator3_Inter_Spec, "cell 0 12 6 1,growx");

		lblDataDisplay_Inter_Spec = new Label("Data Display");
		lblDataDisplay_Inter_Spec.setFont(new Font("Dialog", Font.BOLD, 12));
		add(lblDataDisplay_Inter_Spec, "cell 0 15,growx");
		
		Label lbl_CommonWVN_Inter_Spec = new Label("Number of Data Points");
		lbl_CommonWVN_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(lbl_CommonWVN_Inter_Spec, "cell 0 17,growx");


		
		cmb_Wavenumber_No_Of_Points_Inter_Spec = new JComboBox<String>();
		cmb_Wavenumber_No_Of_Points_Inter_Spec.setEnabled(false);
		cmb_Wavenumber_No_Of_Points_Inter_Spec.setFont(new Font("Dialog", Font.BOLD, 10));
		cmb_Wavenumber_No_Of_Points_Inter_Spec.setPreferredSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_Optical_Settings_Inter_Spec
				.getPreferredSize().height));
		cmb_Wavenumber_No_Of_Points_Inter_Spec.setMaximumSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_Optical_Settings_Inter_Spec
				.getPreferredSize().height));

		cmb_Wavenumber_No_Of_Points_Inter_Spec.addPopupMenuListener(new PopupMenuListener() {
			public void popupMenuCanceled(PopupMenuEvent e) {
			}

			public void popupMenuWillBecomeInvisible(PopupMenuEvent e) {
				SpectroscopyPanel.cmb_Wavenumber_No_Of_Points_Spec.setSelectedIndex(cmb_Wavenumber_No_Of_Points_Inter_Spec.getSelectedIndex());
			}

			public void popupMenuWillBecomeVisible(PopupMenuEvent e) {
				
			}
		});

		add(cmb_Wavenumber_No_Of_Points_Inter_Spec, "cell 2 17 4 1,growx");
		
		
		chb_EnableLinearInterpolation_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		this.add(chb_EnableLinearInterpolation_Inter_Spec, "cell 0 16 4 1,growx");
		chb_EnableLinearInterpolation_Inter_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {

				SpectroscopyPanel.chb_EnableLinearInterpolation_Spectroscopy.setSelected(chb_EnableLinearInterpolation_Inter_Spec.isSelected());
				
				if(chb_EnableLinearInterpolation_Inter_Spec.isSelected())
				{
					cmb_Wavenumber_No_Of_Points_Inter_Spec.setEnabled(true);
					UserInterface.displayCommonWVN();
					cmb_Wavenumber_No_Of_Points_Inter_Spec.setSelectedIndex(4);
					
					SpectroscopyPanel.cmb_Wavenumber_No_Of_Points_Spec.setEnabled(true);
					SpectroscopyPanel.cmb_Wavenumber_No_Of_Points_Spec.setSelectedIndex(4);
				}
				else
				{
					cmb_Wavenumber_No_Of_Points_Inter_Spec.setEnabled(false);
					cmb_Wavenumber_No_Of_Points_Inter_Spec.removeAllItems();
					
					SpectroscopyPanel.cmb_Wavenumber_No_Of_Points_Spec.setEnabled(false);
					SpectroscopyPanel.cmb_Wavenumber_No_Of_Points_Spec.removeAllItems();
				}
				
			}
		});
		
		rdbtn_nm_Inter_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {

				remove(spectrumChartPanel);
				spectrumChartPanel = null;

				spectrumChartPanel = spectrumChart_NM.getChartPanel();
				spectrumChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));

				//Remove legend of the fake data plotted to chart if no valid data was plotted
				if(measurementCount_Spectrum == 0)
				{
					spectrumChartPanel.getChart().removeLegend();
				}

				spectrumChartPanel.repaint();

				spectrumChartPanel.setPreferredSize(new Dimension( (int) spectrumChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

				add(spectrumChartPanel, "cell 6 0 10 21, grow");

				revalidate();
				repaint();
			}
		});

		ButtonGroup bgDataDisplay_X_Axis = new ButtonGroup();

		rdbtn_nm_Inter_Spec.setSelected(true);
		rdbtn_nm_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(rdbtn_nm_Inter_Spec, "flowx,cell 1 18 2 1,aligny top");
		bgDataDisplay_X_Axis.add(rdbtn_nm_Inter_Spec);
		rdbtn_cm_Inter_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {

				remove(spectrumChartPanel);
				spectrumChartPanel = null;

				spectrumChartPanel = spectrumChart_CM.getChartPanel();
				spectrumChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));

				//Remove legend of the fake data plotted to chart if no valid data was plotted
				if(measurementCount_Spectrum == 0)
				{
					spectrumChartPanel.getChart().removeLegend();
				}

				spectrumChartPanel.repaint();

				spectrumChartPanel.setPreferredSize(new Dimension( (int) spectrumChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

				add(spectrumChartPanel, "cell 6 0 10 21, grow");

				revalidate();
				repaint();
			}
		});

		rdbtn_cm_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(rdbtn_cm_Inter_Spec, "cell 3 18 3 1,aligny top");
		bgDataDisplay_X_Axis.add(rdbtn_cm_Inter_Spec);

		lblXaxis_Inter_Spec = new Label("X-Axis:");
		lblXaxis_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(lblXaxis_Inter_Spec, "cell 0 18,growx,aligny top");

		
		Label lblYaxis_Inter_Spec = new Label("Y-Axis:");
		lblYaxis_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		lblYaxis_Inter_Spec.setVisible(false);
		this.add(lblYaxis_Inter_Spec, "cell 0 19,growx,aligny top");
		
		JRadioButton rdbtn_Refl_Trans = new JRadioButton("%Refl. or Trans.");
		rdbtn_Refl_Trans.setFont(new Font("Dialog", Font.PLAIN, 10));
		rdbtn_Refl_Trans.setVisible(false);
		this.add(rdbtn_Refl_Trans, "flowx,cell 1 19 2 1,aligny top");
		
		JRadioButton rdbtn_Absorbance = new JRadioButton("Abs");
		rdbtn_Absorbance.setFont(new Font("Dialog", Font.PLAIN, 10));
		rdbtn_Absorbance.setVisible(false);
		this.add(rdbtn_Absorbance, "cell 3 19 3 1,aligny top");

		JSeparator separator4_Inter_Spec = new JSeparator();
		this.add(separator4_Inter_Spec, "cell 0 20 6 1,growx");

		lbl_advancedSettings_Inter_Spec = new Label("Advanced Settings");
		lbl_advancedSettings_Inter_Spec.setVisible(false);
		lbl_advancedSettings_Inter_Spec.setFont(new Font("Dialog", Font.BOLD, 12));
		this.add(lbl_advancedSettings_Inter_Spec, "cell 6 22 2 1,growx");

		lbl_FFT_Settings_Inter_Spec = new Label("FFT Settings");
		lbl_FFT_Settings_Inter_Spec.setVisible(false);
		lbl_FFT_Settings_Inter_Spec.setFont(new Font("Dialog", Font.BOLD, 12));
		this.add(lbl_FFT_Settings_Inter_Spec, "cell 12 22,growx");

		btn_GainAdjustmentPanel_Inter_Spec = new JButton("");
		btn_GainAdjustmentPanel_Inter_Spec.setVisible(false);
		btn_GainAdjustmentPanel_Inter_Spec.setEnabled(false);
		btn_GainAdjustmentPanel_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_GainAdjustmentPanel_Inter_Spec.setText("<html><center>Add New Optical Gain Setting</center></html>");
		btn_GainAdjustmentPanel_Inter_Spec.setVerticalTextPosition(SwingConstants.CENTER);
		btn_GainAdjustmentPanel_Inter_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_GainAdjustmentPanel_Inter_Spec.setToolTipText("Add new optical gain setting");

		btn_GainAdjustmentPanel_Inter_Spec.addActionListener(new ActionListener() {
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
		this.add(btn_GainAdjustmentPanel_Inter_Spec, "cell 6 23 2 1,grow");

		btn_WavelengthWavenumberCalibration_Inter_Spec = new JButton("");
		btn_WavelengthWavenumberCalibration_Inter_Spec.setVisible(false);
		btn_WavelengthWavenumberCalibration_Inter_Spec.setEnabled(false);
		btn_WavelengthWavenumberCalibration_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_WavelengthWavenumberCalibration_Inter_Spec.setText("<html><center>Wavelength Correction</center></html>");
		btn_WavelengthWavenumberCalibration_Inter_Spec.setVerticalTextPosition(SwingConstants.CENTER);
		btn_WavelengthWavenumberCalibration_Inter_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_WavelengthWavenumberCalibration_Inter_Spec.setToolTipText("Wavelength & Wavenumber Correction");

		btn_WavelengthWavenumberCalibration_Inter_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				UserInterface.stopEnablingButtons = true;
				UserInterface.boardReadyRoutine(false);
				try{
					int commonWavenumberSelection = 0;
					if(chb_EnableLinearInterpolation_Inter_Spec.isSelected())
					{
						commonWavenumberSelection = cmb_Wavenumber_No_Of_Points_Inter_Spec.getSelectedIndex() + 1;
					}

					WL_WN_CalibrationPanel calibratoinPanel = new WL_WN_CalibrationPanel(
							String.valueOf(Double.parseDouble(txt_RunTime_Inter_Spec.getText()) * 1000),
							"",
							UserInterface.getOpticalGainValue((String)cmb_Optical_Settings_Inter_Spec.getSelectedItem()),
							String.valueOf(cmb_Apodization_Inter_Spec.getSelectedIndex()),
							String.valueOf(cmb_ZeroPadding_Inter_Spec.getSelectedIndex()),
							String.valueOf(commonWavenumberSelection),
							p3Constants.InterSpecPrefix);
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
		this.add(btn_WavelengthWavenumberCalibration_Inter_Spec, "cell 8 23 2 1,grow");

		lbl_Apodization_Inter_Spec = new Label("Apodization Function");
		lbl_Apodization_Inter_Spec.setVisible(false);
		lbl_Apodization_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(lbl_Apodization_Inter_Spec, "cell 12 23,growx");

		JLabel lbl_logo_Inter_Spec = new JLabel("");
		lbl_logo_Inter_Spec.setFont(new Font("Dialog", Font.BOLD, 10));
		lbl_logo_Inter_Spec.setForeground(Color.WHITE);
		lbl_logo_Inter_Spec.setBackground(Color.WHITE);
		this.add(lbl_logo_Inter_Spec, "cell 0 22 6 3,alignx left,aligny bottom");

		// Plot fake data to let the panel appeared
		spectrumChart_NM = new XYLineChart("Power Spectral Density (PSD)", "", "Wavelength (nm)", "PSD (a.u.)", new double[][] { new double[] { 0 }, new double[] { 0 } }, PSD_X_NM_MIN, PSD_X_NM_MAX, PSD_Y_MIN, PSD_Y_MAX);
		spectrumChart_CM = new XYLineChart("Power Spectral Density (PSD)", "", "Wavenumber (cm -\u00B9)", "PSD (a.u.)", new double[][] { new double[] { 0 }, new double[] { 0 } }, PSD_X_CM_MIN, PSD_X_CM_MAX, PSD_Y_MIN, PSD_Y_MAX);

		spectrumChartPanel = spectrumChart_NM.getChartPanel();

		spectrumChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));

		//Remove legend of the fake data plotted to chart
		spectrumChartPanel.getChart().removeLegend();

		spectrumChartPanel.setPreferredSize(new Dimension( (int) spectrumChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

		this.add(spectrumChartPanel, "cell 6 0 10 21, grow");

		Label lbl_Graphs_Inter_Spec = new Label("Plots");
		lbl_Graphs_Inter_Spec.setFont(new Font("Dialog", Font.BOLD, 12));
		this.add(lbl_Graphs_Inter_Spec, "cell 0 9,growx");

		btn_ClearGraphs_Inter_Spec = new JButton("");
		btn_ClearGraphs_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_ClearGraphs_Inter_Spec.setText("Clear");
		btn_ClearGraphs_Inter_Spec.setVerticalTextPosition(SwingConstants.BOTTOM);
		btn_ClearGraphs_Inter_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_ClearGraphs_Inter_Spec.setToolTipText("Clear Plots");

		btn_ClearGraphs_Inter_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {

				clearingGraphsInProgress = true;

				p3AppManagerUtils.createDir(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
				File folder = new File(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
				File[] listOfFiles = folder.listFiles();
				for (File file : listOfFiles) {
					if (file.getAbsolutePath().endsWith(".Interferogram") || file.getAbsolutePath().endsWith(".InterPSD")) {
						file.delete();
					}
				}

				// Plot fake data to let the panel appeared
				spectrumChart_NM = new XYLineChart("Power Spectral Density (PSD)", "", "Wavelength (nm)", "PSD (a.u.)", new double[][] { new double[] { 0 }, new double[] { 0 } }, PSD_X_NM_MIN, PSD_X_NM_MAX, PSD_Y_MIN, PSD_Y_MAX);
				spectrumChart_CM = new XYLineChart("Power Spectral Density (PSD)", "", "Wavenumber (cm -\u00B9)", "PSD (a.u.)", new double[][] { new double[] { 0 }, new double[] { 0 } }, PSD_X_CM_MIN, PSD_X_CM_MAX, PSD_Y_MIN, PSD_Y_MAX);

				remove(spectrumChartPanel);

				spectrumChartPanel = null;

				spectrumChartPanel = spectrumChart_NM.getChartPanel();
				rdbtn_nm_Inter_Spec.setSelected(true);

				spectrumChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));

				//Remove legend of the fake data plotted to chart
				spectrumChartPanel.getChart().removeLegend();

				spectrumChartPanel.repaint();

				spectrumChartPanel.setPreferredSize(new Dimension( (int) spectrumChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

				add(spectrumChartPanel, "cell 6 0 10 21, grow");

				measurementCount_Spectrum = 0;

				UserInterface.frmMain.revalidate();
				UserInterface.frmMain.repaint();

				AutoSavePath_Inter_Spec = "";
				if (chb_AutoSave_Inter_Spec.isSelected()) {
					chb_AutoSave_Inter_Spec.setSelected(false);
					JOptionPane.showMessageDialog(
							null,
							"Auto-save is disabled, please rechoose saving directory",
							"Auto-save Disabled!",
							JOptionPane.OK_OPTION);
				}

				clearingGraphsInProgress = false;
			}
		});
		this.add(btn_ClearGraphs_Inter_Spec, "cell 3 10 3 1,growx");

		chb_AutoSave_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		chb_AutoSave_Inter_Spec.addActionListener(new ActionListener() {
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

				if (chb_AutoSave_Inter_Spec.isSelected()) {
					if (autoSaveSpec.showOpenDialog(null) == JFileChooser.APPROVE_OPTION) {
						AutoSavePath_Inter_Spec = autoSaveSpec.getSelectedFile().getAbsolutePath();
						UserInterface.defaultDirectorySaveLoad = AutoSavePath_Inter_Spec;
					} else {
						chb_AutoSave_Inter_Spec.setSelected(false);
					}
				} else {
					AutoSavePath_Inter_Spec = "";
				}
			}
		});

		this.add(chb_AutoSave_Inter_Spec, "cell 1 9 2 1,growx");

		separatorV1_Inter_Spec = new JSeparator();
		separatorV1_Inter_Spec.setVisible(false);
		separatorV1_Inter_Spec.setOrientation(SwingConstants.VERTICAL);
		add(separatorV1_Inter_Spec, "cell 11 22 1 5,growy");

		btn_WriteSettings_Inter_Spec = new JButton("");
		btn_WriteSettings_Inter_Spec.setVisible(false);
		btn_WriteSettings_Inter_Spec.setEnabled(false);
		btn_WriteSettings_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		btn_WriteSettings_Inter_Spec.setText("Burn Settings");
		btn_WriteSettings_Inter_Spec.setVerticalTextPosition(SwingConstants.CENTER);
		btn_WriteSettings_Inter_Spec.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_WriteSettings_Inter_Spec.setToolTipText("Burn current settings in kit's ROM");

		btn_WriteSettings_Inter_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {

				UserInterface.burnSequence();
			}
		});
		this.add(btn_WriteSettings_Inter_Spec, "cell 6 24 2 1,growx");

		cmb_Apodization_Inter_Spec = new JComboBox<String>();
		cmb_Apodization_Inter_Spec.setEnabled(false);
		cmb_Apodization_Inter_Spec.setVisible(false);
		cmb_Apodization_Inter_Spec.setFont(new Font("Dialog", Font.BOLD, 10));
		cmb_Apodization_Inter_Spec.setModel(new DefaultComboBoxModel<String>(p3Constants.apodizationOptions));
		cmb_Apodization_Inter_Spec.setMaximumSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_Apodization_Inter_Spec.getPreferredSize().height));
		cmb_Apodization_Inter_Spec.setMinimumSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_Apodization_Inter_Spec.getPreferredSize().height));
		cmb_Apodization_Inter_Spec.setPreferredSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_Apodization_Inter_Spec.getPreferredSize().height));
		this.add(cmb_Apodization_Inter_Spec, "cell 13 23 3 1,growx");

		cmb_Apodization_Inter_Spec.addPopupMenuListener(new PopupMenuListener() {
			public void popupMenuCanceled(PopupMenuEvent e) {
			}

			public void popupMenuWillBecomeInvisible(PopupMenuEvent e) {
				SpectroscopyPanel.cmb_Apodization_Spec.setSelectedIndex(cmb_Apodization_Inter_Spec.getSelectedIndex());
			}

			public void popupMenuWillBecomeVisible(PopupMenuEvent e) {
				cmb_Apodization_Inter_Spec.setModel(new DefaultComboBoxModel<String>(p3Constants.apodizationOptions));
				genuineAction = true;
			}
		});
		cmb_Apodization_Inter_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				File loadedfile = null;
				if(genuineAction)
				{
					genuineAction = false;
					long[] apodizationWindowCoefficients = null;
					
					if(cmb_Apodization_Inter_Spec.getSelectedItem().toString().startsWith("External Function"))
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
								cmb_Apodization_Inter_Spec.setSelectedIndex(0);
								SpectroscopyPanel.cmb_Apodization_Spec.setSelectedIndex(cmb_Apodization_Inter_Spec.getSelectedIndex());
								JOptionPane.showMessageDialog(
										null,
										"Invalid apodization window. \nPlease select a valid window coefficients between 1~20 coefficients",
										"Apodization window", JOptionPane.OK_OPTION);
								apodizationWindowCoefficients = null;
								return;
							}

							UserInterface.defaultDirectorySaveLoad = openWindowFile.getSelectedFile().getParentFile().getAbsolutePath();
						}
					}

					UserInterface.applicationManager.setExternalApodizationWindow(apodizationWindowCoefficients);
				}
				if(loadedfile != null){
					String fileName = loadedfile.getName().replace(".txt", "");
					
					cmb_Apodization_Inter_Spec.setSelectedIndex(0);
					cmb_Apodization_Inter_Spec.removeItemAt(cmb_Apodization_Inter_Spec.getItemCount() - 1);
					cmb_Apodization_Inter_Spec.addItem(fileName);
					cmb_Apodization_Inter_Spec.setSelectedIndex(cmb_Apodization_Inter_Spec.getItemCount() - 1);
					
					SpectroscopyPanel.cmb_Apodization_Spec.removeItemAt(SpectroscopyPanel.cmb_Apodization_Spec.getItemCount() - 1);
					SpectroscopyPanel.cmb_Apodization_Spec.addItem(fileName);
					SpectroscopyPanel.cmb_Apodization_Spec.setSelectedIndex(SpectroscopyPanel.cmb_Apodization_Spec.getItemCount() - 1);
				}else{
					SpectroscopyPanel.cmb_Apodization_Spec.setSelectedIndex(cmb_Apodization_Inter_Spec.getSelectedIndex());
				}
			}
		});
		lbl_ZeroPadding_Inter_Spec = new Label("Zero Padding");
		lbl_ZeroPadding_Inter_Spec.setVisible(false);
		lbl_ZeroPadding_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(lbl_ZeroPadding_Inter_Spec, "cell 12 24,growx");

		cmb_ZeroPadding_Inter_Spec = new JComboBox<String>();
		cmb_ZeroPadding_Inter_Spec.setEnabled(false);
		cmb_ZeroPadding_Inter_Spec.setVisible(false);
		cmb_ZeroPadding_Inter_Spec.setFont(new Font("Dialog", Font.BOLD, 10));
		for(int f = 0; f < p3Constants.paddingOptions.length; f++)
		{
			cmb_ZeroPadding_Inter_Spec.addItem(p3Constants.paddingOptions[f] + " N");
		}
		cmb_ZeroPadding_Inter_Spec.setMaximumSize(new Dimension(
				p3Constants.MAX_WIDTH_OF_FIELD, cmb_ZeroPadding_Inter_Spec.getPreferredSize().height));
		this.add(cmb_ZeroPadding_Inter_Spec, "cell 13 24 3 1,growx");

		cmb_ZeroPadding_Inter_Spec.addPopupMenuListener(new PopupMenuListener() {
			public void popupMenuCanceled(PopupMenuEvent e) {
			}

			public void popupMenuWillBecomeInvisible(PopupMenuEvent e) {
				SpectroscopyPanel.cmb_ZeroPadding_Spec.setSelectedIndex(cmb_ZeroPadding_Inter_Spec.getSelectedIndex());
			}

			public void popupMenuWillBecomeVisible(PopupMenuEvent e) {
			}
		});

		btn_RestoreDefault_Inter_Spec = new JButton("Restore Default Settings");
		btn_RestoreDefault_Inter_Spec.setVisible(false);
		btn_RestoreDefault_Inter_Spec.setToolTipText("Restore Default Settings");
		btn_RestoreDefault_Inter_Spec.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				UserInterface.restoreSequence();
			}
		});
		btn_RestoreDefault_Inter_Spec.setEnabled(false);
		btn_RestoreDefault_Inter_Spec.setFont(new Font("Dialog", Font.PLAIN, 10));
		add(btn_RestoreDefault_Inter_Spec, "cell 8 24 2 1,growx");

		try {
			File imgFile = new File(UserInterface.DELETE_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SMALL_SCALE, -1, Image.SCALE_SMOOTH);
				btn_DeleteOpticalOption_Inter_Spec.setIcon(new ImageIcon(img));
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
			File imgFile = new File(UserInterface.LOAD_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SCALE, -1, Image.SCALE_SMOOTH);
				btn_LoadGraphs_Inter_Spec.setIcon(new ImageIcon(img));
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
				btn_Capture_Interfero.setIcon(new ImageIcon(img));
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
				lbl_logo_Inter_Spec.setHorizontalTextPosition(SwingConstants.LEFT);
				lbl_logo_Inter_Spec.setText("   ");
				lbl_logo_Inter_Spec.setIcon(new ImageIcon(img));
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
			File imgFile = new File(UserInterface.SAVE_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SCALE, -1, Image.SCALE_SMOOTH);
				btn_SaveGraphs_Inter_Spec.setIcon(new ImageIcon(img));
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
			File imgFile = new File(UserInterface.STOP_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SCALE, -1, Image.SCALE_SMOOTH);
				btn_Stop_Inter_Spec.setIcon(new ImageIcon(img));
			} else {
				JOptionPane.showMessageDialog(null,
						"Stop image doesn't exist.", "Loading image",
						JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(null, "Error loading Stop image:"
					+ ex.getMessage(), "Loading image", JOptionPane.OK_OPTION);
		}
		try {
			File imgFile = new File(UserInterface.RUN_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SCALE, -1, Image.SCALE_SMOOTH);
				btn_Run_Inter_Spec.setIcon(new ImageIcon(img));
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
			File imgFile = new File(UserInterface.DELETE_IMAGE_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				Image original = img;
				img = original.getScaledInstance(UserInterface.SCALE, -1, Image.SCALE_SMOOTH);
				btn_ClearGraphs_Inter_Spec.setIcon(new ImageIcon(img));
			} else {
				JOptionPane.showMessageDialog(null,
						"Clear plots image doesn't exist.", "Loading image",
						JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(null,
					"Error loading clear plots image:" + ex.getMessage(),
					"Loading image", JOptionPane.OK_OPTION);
		}
	}

	public void update(Object arg1) {
		if (arg1 instanceof p3AppManagerNotification) {
			logger.info("update method started");
			p3AppManagerNotification resp = (p3AppManagerNotification) arg1;
			
			switch (resp.getAction()) {
			case 1: // Interferogram & Spectrum Run
				logger.info("Run_Inter_Spec Finished");
				if (!UserInterface.stopContinuousRun) {
					if (resp.getStatus() == 0) {
						logger.info("Get Data of Run_Inter_Spec Started");
						double[][] data = UserInterface.applicationManager.getInterSpecData();
						logger.info("Get Data of Run_Inter_Spec Finished");
						double[] xInverse = UserInterface.switch_NM_CM(data[p3Constants.WAVENUMBER_INDEX]);
						boolean	clearedGraph_Spectrum = false; 


						if(!chb_EnableLinearInterpolation_Inter_Spec.isSelected())
						{
							String str;
							
							str = String.valueOf((int)data[4][0]);
							str = str + " pts";
							
							InterSpecPanel.cmb_Wavenumber_No_Of_Points_Inter_Spec.removeAllItems();
							SpectroscopyPanel.cmb_Wavenumber_No_Of_Points_Spec.removeAllItems();
							
							InterSpecPanel.cmb_Wavenumber_No_Of_Points_Inter_Spec.addItem(str);
							SpectroscopyPanel.cmb_Wavenumber_No_Of_Points_Spec.addItem(str);
						}
						
						if(clearingGraphsInProgress == false)
						{
							//Clear plots is not running now

							// Drawing Charts
							try {
								if(measurementCount_Spectrum == 0)
									clearedGraph_Spectrum = true;

								// removes last plot if user didn't capture it.
								if ((!capture_Displayed_InterSpec) && (measurementCount_Spectrum != 0)) {
									measurementCount_Spectrum--;
								}

								measurementCount_Spectrum++;
								if (measurementCount_Spectrum == 1 && clearedGraph_Spectrum) {
									spectrumChart_NM = new XYLineChart("Power Spectral Density (PSD)", "Measurement No. " + measurementCount_Spectrum, "Wavelength (nm)", "PSD (a.u.)", new double[][] { xInverse, data[3] }, true);
									spectrumChart_CM = new XYLineChart("Power Spectral Density (PSD)", "Measurement No. " + measurementCount_Spectrum, "Wavenumber (cm -\u00B9)", "PSD (a.u.)", new double[][] { data[2], data[3] }, true);
									clearedGraph_Spectrum = false;
								} else {

									spectrumChart_NM.addSeries("Measurement No. " + measurementCount_Spectrum, new double[][] { xInverse, data[3] }, true);
									spectrumChart_CM.addSeries("Measurement No. " + measurementCount_Spectrum, new double[][] { data[2], data[3] }, true);
								}

								capture_Displayed_InterSpec = false;

								remove(spectrumChartPanel);

								spectrumChartPanel = null;

								if(rdbtn_nm_Inter_Spec.isSelected())
								{
									spectrumChartPanel = spectrumChart_NM.getChartPanel();
								}
								else
								{
									spectrumChartPanel = spectrumChart_CM.getChartPanel();
								}

								spectrumChartPanel.setBorder(new LineBorder( new Color(0, 0, 0), 1, true));

								spectrumChartPanel.setPreferredSize(new Dimension( (int) spectrumChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

								add(spectrumChartPanel, "cell 6 0 10 21, grow");

								spectrumChartPanel.repaint();
							} catch (Exception e) {
								logger.error(e.getMessage());
							}
							// //////////////////////////////////////////////////////////////////

							if (data != null) {
								logger.info("writing plot to file started");
								writeRunDataFile(data, resp.getDeviceId());
								logger.info("writing plot to file finished");
							}
						}
						else
						{
							//Ignore plotting
						}

						data = null;
						xInverse = null;
						if (UserInterface.stopContinuousRun || !UserInterface.isContinuousRun || !rdbtn_ContRun_Inter_Spec.isSelected()) {

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
								UserInterface.convertErrorCodesToMessages(resp.getStatus()),
								"Run NeoSpectra Micro", JOptionPane.OK_OPTION);
						btn_Stop_Inter_Spec.doClick();
					}
				}

				//cont run
				if (!UserInterface.stopContinuousRun && UserInterface.isContinuousRun && rdbtn_ContRun_Inter_Spec.isSelected()) {
					UserInterface.releaseDeviceToken();
					RunSequence();
					try{logger.info("Run_Inter_Spec Finished (Without PowerUp) - Scan Time = " + String.valueOf(Double.parseDouble(txt_RunTime_Inter_Spec.getText()) * 1000));}catch(Exception d){}
				}
				else //single run
				{
					if(rdbtn_ContRun_Inter_Spec.isSelected())
					{
						UserInterface.colorLabel.setForeground(Color.green.darker());
						UserInterface.colorLabel.setBackground(Color.green.darker());
					}

					logger.info("enabling GUI fields started");
					UserInterface.boardReadyRoutine(true);
					logger.info("enabling GUI fields finished");

					boolean btn_capture_status = btn_Capture_Interfero.isEnabled();
					btn_Capture_Interfero.setEnabled(true);
					btn_Capture_Interfero.doClick();
					btn_Capture_Interfero.setEnabled(btn_capture_status);

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

	boolean writeRunDataFile(double[][] data, String moduleId) {

		double[] xInverse = UserInterface.switch_NM_CM(data[p3Constants.WAVENUMBER_INDEX]);

		if(rdbtn_nm_Inter_Spec.isSelected())
		{
			// PSD file
			if (!UserInterface.writeGraphFile(xInverse, data[p3Constants.POWER_SPECTRAL_DENSITY_INDEX], p3AppManagerUtils.formatString(p3Constants.getPath(PSD_PATH_TEMPLATE), measurementCount_Spectrum), PSD_FILE_X_AXIS_NM + "\t" + PSD_FILE_Y_AXIS)) {

				throw new
				p3AppManagerException("Error writing write PSD file    ",
						p3AppManagerStatus.PSD_FILE_CREATION_ERROR.getNumVal());

			}
		}
		else
		{
			// PSD file
			if (!UserInterface.writeGraphFile(data[p3Constants.WAVENUMBER_INDEX], data[p3Constants.POWER_SPECTRAL_DENSITY_INDEX], p3AppManagerUtils.formatString(p3Constants.getPath(PSD_PATH_TEMPLATE), measurementCount_Spectrum), PSD_FILE_X_AXIS_CM + "\t" + PSD_FILE_Y_AXIS)) {

				throw new
				p3AppManagerException("Error while writing PSD file    ",
						p3AppManagerStatus.PSD_FILE_CREATION_ERROR.getNumVal());

			}
		}

		xInverse = null;

		return true;
	}

	private void RunSequence()
	{

		UserInterface.isContinuousRun = true;
		UserInterface.stopContinuousRun = false;
		logger.info("RunSequence function started");
		// Start RUN
		if (!UserInterface.stopContinuousRun) {
			try{
				// stop checking the status of the device
				UserInterface.checkDeviceStatusThreadStop = true;
				
				int commonWavenumberSelection = 0;
				if(chb_EnableLinearInterpolation_Inter_Spec.isSelected())
				{
					commonWavenumberSelection = cmb_Wavenumber_No_Of_Points_Inter_Spec.getSelectedIndex() + 1;
				}

				UserInterface.progressTime = Double.parseDouble(txt_RunTime_Inter_Spec.getText().toString()) * 1000;
				UserInterface.progressPar.setValue(0);
				logger.info("runInterSpec function started");
				while(UserInterface.acquireDeviceToken() == false);
				p3AppManagerStatus status = UserInterface.applicationManager.runInterSpec(
						String.valueOf(Double.parseDouble(txt_RunTime_Inter_Spec.getText()) * 1000),
						Integer.toString(cmb_Apodization_Inter_Spec.getSelectedIndex()),
						Integer.toString(cmb_ZeroPadding_Inter_Spec.getSelectedIndex()), UserInterface.getOpticalGainValue((String)cmb_Optical_Settings_Inter_Spec.getSelectedItem()),
						String.valueOf(commonWavenumberSelection), Integer.toString(rdbtn_ContRun_Inter_Spec.isSelected()? 1:0));

				logger.info("runInterSpec function finished");
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
					logger.info("disabling GUI fields started");
					UserInterface.boardReadyRoutine(false);
					logger.info("disabling GUI fields finished");
					UserInterface.statusLabel.setText("Run started. Please wait...");

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

	private void LoadCharts(File loadedFile)
	{

		double[][] arraysToPlot = UserInterface.loadGraphDataFromFile(loadedFile.getAbsolutePath());
		if(arraysToPlot == null)
			return;
		try {
			if (loadedFile.getAbsolutePath().endsWith(".InterPSD")) {
				remove(spectrumChartPanel);
				measurementCount_Spectrum++;

				double[] xInverse = UserInterface.switch_NM_CM(arraysToPlot[0]);

				if (measurementCount_Spectrum == 1) {
					if(rdbtn_nm_Inter_Spec.isSelected())
					{
						spectrumChart_NM = new XYLineChart("Power Spectral Density (PSD)", "Measurement No. " + measurementCount_Spectrum, "Wavelength (nm)", "PSD (a.u.)", arraysToPlot);
						spectrumChart_CM = new XYLineChart("Power Spectral Density (PSD)", "Measurement No. " + measurementCount_Spectrum, "Wavenumber (cm -\u00B9)", "PSD (a.u.)", new double[][] { xInverse, arraysToPlot[1] });
					}
					else
					{
						spectrumChart_CM = new XYLineChart("Power Spectral Density (PSD)", "Measurement No. " + measurementCount_Spectrum, "Wavenumber (cm -\u00B9)", "PSD (a.u.)", arraysToPlot);
						spectrumChart_NM = new XYLineChart("Power Spectral Density (PSD)", "Measurement No. " + measurementCount_Spectrum, "Wavelength (nm)", "PSD (a.u.)", new double[][] { xInverse, arraysToPlot[1] });
					}
				} else {
					if(rdbtn_nm_Inter_Spec.isSelected())
					{
						spectrumChart_NM.addSeries("Measurement No. " + measurementCount_Spectrum, arraysToPlot);
						spectrumChart_CM.addSeries("Measurement No. " + measurementCount_Spectrum, new double[][] { xInverse, arraysToPlot[1] });
					}
					else
					{
						spectrumChart_CM.addSeries("Measurement No. " + measurementCount_Spectrum, arraysToPlot);
						spectrumChart_NM.addSeries("Measurement No. " + measurementCount_Spectrum, new double[][] { xInverse, arraysToPlot[1] });
					}
				}

				if(rdbtn_nm_Inter_Spec.isSelected())
				{
					spectrumChartPanel = spectrumChart_NM.getChartPanel();
				}
				else
				{
					spectrumChartPanel = spectrumChart_CM.getChartPanel();
				}
				spectrumChartPanel.setBorder(new LineBorder(new Color(0, 0, 0), 1, true));
				spectrumChartPanel.repaint();

				spectrumChartPanel.setPreferredSize(new Dimension( (int) spectrumChartPanel.getPreferredSize().getWidth(), (int) UserInterface.screenSize.getHeight()));

				add(spectrumChartPanel, "cell 6 0 10 21, grow");

				// copy the plot file to temp plots directory
				p3AppManagerUtils.createDir(p3Constants.getPath(UserInterface.GRAPH_FILES_FOLDER_PATH));
				File folder = new File(p3AppManagerUtils.formatString(p3Constants.getPath(PSD_PATH_TEMPLATE), measurementCount_Spectrum));

				Files.copy(loadedFile.toPath(), folder.toPath(), StandardCopyOption.REPLACE_EXISTING);
			}

			boolean btn_capture_status = btn_Capture_Interfero.isEnabled();
			btn_Capture_Interfero.setEnabled(true);
			btn_Capture_Interfero.doClick();
			btn_Capture_Interfero.setEnabled(btn_capture_status);

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

				p3AppManagerUtils.createDir(saveSpecFile.getSelectedFile().getAbsolutePath());
				int overwriteRemember = -1;

				for (int i = 0; i < finalFileNames.length; i++) {
					if (listOfFiles[i].isFile()) {
						if (listOfFiles[i].getAbsolutePath().endsWith(".Interferogram") || listOfFiles[i].getAbsolutePath().endsWith(".InterPSD")) {
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
											Files.copy(listOfFiles[i].toPath(), (new File(saveSpecFile.getSelectedFile().getAbsolutePath() + File.separatorChar + finalFileNames[i])).toPath(), StandardCopyOption.REPLACE_EXISTING);
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
										Files.copy(listOfFiles[i].toPath(), (new File(saveSpecFile.getSelectedFile().getAbsolutePath() + File.separatorChar + finalFileNames[i])).toPath(), StandardCopyOption.REPLACE_EXISTING);
									}
								}
								//no overwriting will happen
								else
								{
									Files.copy(listOfFiles[i].toPath(), (new File(saveSpecFile.getSelectedFile().getAbsolutePath() + File.separatorChar + finalFileNames[i])).toPath(), StandardCopyOption.REPLACE_EXISTING);
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
