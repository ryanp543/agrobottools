package sws.spectromost;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Image;
import java.awt.Label;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.BorderFactory;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.SwingConstants;
import javax.swing.border.Border;
import javax.swing.border.EtchedBorder;
import javax.swing.event.PopupMenuEvent;
import javax.swing.event.PopupMenuListener;
import javax.swing.JLabel;

import net.miginfocom.swing.MigLayout;

import org.apache.log4j.Logger;

import sws.p3AppManager_micro.utils.p3AppManagerNotification;
import sws.p3AppManager_micro.utils.p3Constants;
import sws.p3AppManager_micro.utils.p3Enumerations.p3AppManagerStatus;

@SuppressWarnings("serial")
public class WL_WN_CalibrationPanel extends JPanel {
	private static Logger logger = Logger.getLogger(WL_WN_CalibrationPanel.class);

	private static String runTime = null;
	private static String resolution = null;
	private static String opticalSettings = null;
	private static String commonWVN = null;
	private static String tabPrefix = null;
	private static String apodizationIndex = null;
	private static String zeroPaddingMultiplier = null;

	static JFrame frmWL_Calib;

	public static JRadioButton rdbtn_Self_Calib = new JRadioButton("Self-Correction");
	public static JRadioButton rdbtn_RM_Corr = new JRadioButton("Correction Using Reference Material");
	static JButton btn_Proceed_WL_Calib;
	static JButton btn_Close_WL_Calib;
	static JLabel lbl_Progress_Details_WL_Calib;
	static JComboBox<String> cmb_StandardCalibrator_WL_Calib;
	static JButton btn_LoadStandardCalibrator_WL_Calib;

	/*
	 * ! This class is used for actions needed in this panel
	 */
	private class WL_WN_Calib_Actions{
		private final String actionName;
		private boolean succeeded;

		WL_WN_Calib_Actions(String actionName) {
			this.actionName = actionName;
			this.succeeded = false;
		}

		public String getActionName() {
			return this.actionName;
		}

		public Boolean getSucceeded(){
			return this.succeeded;
		}

		public void setSucceeded(boolean succeeded){
			this.succeeded = succeeded;
		}
	}

	private static String[] panelActionNamesSelfCalib = new String[]{"Step 1: Reference light measurement", "Step 2: Process self-correction"};
	private static WL_WN_Calib_Actions[] panelActionsSelfCalib = new WL_WN_Calib_Actions[panelActionNamesSelfCalib.length];

	private static String[] panelActionNamesRM = new String[]{"Step 1: Reference light measurement", "Step 2: Reference material measurement", "Step 3: Process self-correction"};
	private static WL_WN_Calib_Actions[] panelActionsRM = new WL_WN_Calib_Actions[panelActionNamesRM.length];

	public WL_WN_CalibrationPanel(String runTime, String resolution, String opticalSettings, String apodizationIndex, String zeroPaddingMultiplier, String commonWVN, String tabPrefix) {
		super();
		WL_WN_CalibrationPanel.runTime = runTime;
		WL_WN_CalibrationPanel.resolution = resolution;
		WL_WN_CalibrationPanel.opticalSettings = opticalSettings;
		WL_WN_CalibrationPanel.apodizationIndex = apodizationIndex;
		WL_WN_CalibrationPanel.zeroPaddingMultiplier = zeroPaddingMultiplier;
		WL_WN_CalibrationPanel.commonWVN = commonWVN;
		WL_WN_CalibrationPanel.tabPrefix = tabPrefix;
		this.initialize();
	}

	public void initialize() {

		frmWL_Calib = new JFrame();
		frmWL_Calib.setAlwaysOnTop (true);

		//Initialize actions array
		for(int i = 0; i < panelActionsSelfCalib.length; i ++){
			panelActionsSelfCalib[i] = new WL_WN_Calib_Actions(panelActionNamesSelfCalib[i]);
		}

		//Initialize actions array
		for(int i = 0; i < panelActionsRM.length; i ++){
			panelActionsRM[i] = new WL_WN_Calib_Actions(panelActionNamesRM[i]);
		}

		try {
			File imgFile = new File(UserInterface.ICON_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				frmWL_Calib.setIconImage(img);
			} else {
				JOptionPane.showMessageDialog(frmWL_Calib, "Icon doesn't exist.",
						"Loading image", JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(frmWL_Calib,
					"Error loading Icon:" + ex.getMessage(), "Loading icon",
					JOptionPane.OK_OPTION);
		}

		frmWL_Calib.setBackground(new Color(176, 196, 222));
		frmWL_Calib.getContentPane().setBackground(new Color(176, 196, 222));
		frmWL_Calib.setVisible(true);
		frmWL_Calib.setName("SpecFrame");

		frmWL_Calib.setMinimumSize(new Dimension((int)(p3Constants.SUB_PANELS_DIMENTION * 1.2), (int)(p3Constants.SUB_PANELS_DIMENTION)));
		frmWL_Calib.setMaximumSize(new Dimension((int)(p3Constants.SUB_PANELS_DIMENTION * 1.2), (int)(p3Constants.SUB_PANELS_DIMENTION)));
		frmWL_Calib.setPreferredSize(new Dimension((int)(p3Constants.SUB_PANELS_DIMENTION * 1.2), (int)(p3Constants.SUB_PANELS_DIMENTION)));
		frmWL_Calib.setLocation((int)(UserInterface.screenSize.getWidth()/2 - p3Constants.SUB_PANELS_DIMENTION / 2), (int)(UserInterface.screenSize.getHeight()/2 - p3Constants.SUB_PANELS_DIMENTION / 2));

		frmWL_Calib.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent arg0) {
				// Exit
				onCloseOperation();
			}
		});
		frmWL_Calib.setTitle("Wavelength & Wavenumber Correction");
		frmWL_Calib.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		frmWL_Calib.setResizable(false);

		frmWL_Calib.setFont(new Font("Dialog", Font.PLAIN, 12));
		frmWL_Calib.setBackground(new Color(176, 196, 222));
		frmWL_Calib.getContentPane().setLayout(new MigLayout("", "[90.00:90.00][45.00:45.00][grow,fill]", "[][][][][][grow][]"));

		rdbtn_Self_Calib.setFont(new Font("Dialog", Font.PLAIN, 12));
		frmWL_Calib.getContentPane().add(rdbtn_Self_Calib, "cell 0 0 1 1,growx");
		rdbtn_Self_Calib.setSelected(true);
		rdbtn_Self_Calib.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				//Reset both operations
				for (int i = 0; i < panelActionsSelfCalib.length; i++) {
					panelActionsSelfCalib[i].setSucceeded(false);
				}

				for (int i = 0; i < panelActionsRM.length; i++) {
					panelActionsRM[i].setSucceeded(false);
				}

				//Display correct steps
				lbl_Progress_Details_WL_Calib.setText(getActionsString());
				if(tabPrefix.equals(p3Constants.SpecPrefix))
				{
					cmb_StandardCalibrator_WL_Calib.setEnabled(false);
					btn_LoadStandardCalibrator_WL_Calib.setEnabled(false);
				}
			}
		});

		if(tabPrefix.equals(p3Constants.SpecPrefix))
		{
			rdbtn_RM_Corr.setFont(new Font("Dialog", Font.PLAIN, 12));
			frmWL_Calib.getContentPane().add(rdbtn_RM_Corr, "cell 0 1 1 1,growx");
			rdbtn_RM_Corr.setSelected(false);
			rdbtn_RM_Corr.addActionListener(new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent arg0) {
					//Reset both operations
					for (int i = 0; i < panelActionsSelfCalib.length; i++) {
						panelActionsSelfCalib[i].setSucceeded(false);
					}

					for (int i = 0; i < panelActionsRM.length; i++) {
						panelActionsRM[i].setSucceeded(false);
					}

					//Display correct steps
					lbl_Progress_Details_WL_Calib.setText(getActionsString());
					cmb_StandardCalibrator_WL_Calib.setEnabled(true);
					btn_LoadStandardCalibrator_WL_Calib.setEnabled(true);
				}
			});
		}

		ButtonGroup selectAction = new ButtonGroup();
		selectAction.add(rdbtn_Self_Calib);
		if(tabPrefix.equals(p3Constants.SpecPrefix))
		{
			selectAction.add(rdbtn_RM_Corr);
		}

		if(tabPrefix.equals(p3Constants.SpecPrefix))
		{
			cmb_StandardCalibrator_WL_Calib = new JComboBox<String>();
			cmb_StandardCalibrator_WL_Calib.setEnabled(false);
			cmb_StandardCalibrator_WL_Calib.setFont(new Font("Dialog", Font.BOLD, 12));
			cmb_StandardCalibrator_WL_Calib.setMaximumSize(new Dimension(
					p3Constants.MAX_WIDTH_OF_FIELD * 2, cmb_StandardCalibrator_WL_Calib.getPreferredSize().height));
			frmWL_Calib.getContentPane().add(cmb_StandardCalibrator_WL_Calib, "cell 0 2 3 1,growx");

			cmb_StandardCalibrator_WL_Calib.addPopupMenuListener(new PopupMenuListener() {
				public void popupMenuCanceled(PopupMenuEvent e) {
				}

				public void popupMenuWillBecomeInvisible(PopupMenuEvent e) {
				}

				public void popupMenuWillBecomeVisible(PopupMenuEvent e) {
					UserInterface.displayStandardCalibrators();
				}
			});

			btn_LoadStandardCalibrator_WL_Calib = new JButton("");
			btn_LoadStandardCalibrator_WL_Calib.setFont(new Font("Dialog", Font.PLAIN, 10));
			btn_LoadStandardCalibrator_WL_Calib.setEnabled(false);
			btn_LoadStandardCalibrator_WL_Calib.setText("Add New");
			btn_LoadStandardCalibrator_WL_Calib.setVerticalTextPosition(SwingConstants.CENTER);
			btn_LoadStandardCalibrator_WL_Calib.setHorizontalTextPosition(SwingConstants.CENTER);
			btn_LoadStandardCalibrator_WL_Calib.setToolTipText("Add New Standard Calibrator");
			btn_LoadStandardCalibrator_WL_Calib.setMaximumSize(new Dimension(p3Constants.MAX_WIDTH_OF_FIELD, btn_LoadStandardCalibrator_WL_Calib.getPreferredSize().height));

			btn_LoadStandardCalibrator_WL_Calib.addActionListener(new ActionListener() {
				public void actionPerformed(ActionEvent arg0) {
					logger.info("NewStandardCalibrator Started");
					String newStandardCalibrator = (String)JOptionPane.showInputDialog(
							frmWL_Calib,
							"Please enter new standard calibrator as:\n"
									+"Standard_Calibrator_Name{wavelength1_nm,wavelength2_nm}\n"
									+"Example: Methylene_Cloride{1367.5,1417.5}\n"
									+ "NOTE: Spaces are not allowed",
									"New Standard Calibrator",
									JOptionPane.PLAIN_MESSAGE,
									null,
									null,
							"Standard_Calibrator_Name{WL1_nm,WL2_nm}");
					if(null == newStandardCalibrator)
						return;

					if(!newStandardCalibrator.equals(""))
					{

						if(UserInterface.addNewStandardCalibrator(newStandardCalibrator))
						{
							JOptionPane.showMessageDialog(frmWL_Calib,
									"New Standard Calibrator Saved Successfully",
									"New Standard Calibrator", JOptionPane.INFORMATION_MESSAGE);
						}else {
							JOptionPane.showMessageDialog(frmWL_Calib,
									"Wrong format. Failed to save!",
									"New Standard Calibrator", JOptionPane.OK_OPTION);
						}

					}
					else 
					{
						JOptionPane.showMessageDialog(frmWL_Calib,
								"Failed to save!",
								"New Standard Calibrator", JOptionPane.OK_OPTION);
					}
				}
			});
			frmWL_Calib.getContentPane().add(btn_LoadStandardCalibrator_WL_Calib, "cell 2 3 1 1,growx");
		}

		Label lbl_Progress_WL_Calib = new Label("Progress");
		lbl_Progress_WL_Calib.setFont(new Font("Dialog", Font.BOLD, 12));
		frmWL_Calib.getContentPane().add(lbl_Progress_WL_Calib, "cell 0 4 3 1,growx");


		lbl_Progress_Details_WL_Calib = new JLabel();
		lbl_Progress_Details_WL_Calib.setFont(new Font("Dialog", Font.PLAIN, 12));
		Border border = (Border) BorderFactory.createEtchedBorder(EtchedBorder.LOWERED);
		lbl_Progress_Details_WL_Calib.setBorder(border);
		lbl_Progress_Details_WL_Calib.setText(getActionsString());
		lbl_Progress_Details_WL_Calib.setVerticalAlignment(JLabel.TOP);
		lbl_Progress_Details_WL_Calib.setVerticalTextPosition(JLabel.TOP);
		lbl_Progress_Details_WL_Calib.setAlignmentY(TOP_ALIGNMENT);
		frmWL_Calib.getContentPane().add(lbl_Progress_Details_WL_Calib, "cell 0 5 3 1,grow");

		btn_Proceed_WL_Calib = new JButton("");
		btn_Proceed_WL_Calib.setFont(new Font("Dialog", Font.PLAIN, 12));
		btn_Proceed_WL_Calib.setText("Proceed >>");
		btn_Proceed_WL_Calib.setVerticalTextPosition(SwingConstants.CENTER);
		btn_Proceed_WL_Calib.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_Proceed_WL_Calib.setToolTipText("Proceed");
		btn_Proceed_WL_Calib.setMaximumSize(new Dimension((int)(p3Constants.MAX_WIDTH_OF_FIELD), btn_Proceed_WL_Calib.getPreferredSize().height));

		btn_Proceed_WL_Calib.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				int currentActionIndex = 0;
				if(rdbtn_RM_Corr.isSelected())
				{
					for (currentActionIndex = 0; currentActionIndex < panelActionsRM.length; currentActionIndex++) {
						if(!panelActionsRM[currentActionIndex].getSucceeded())
							break;
					}
				}
				else {
					for (currentActionIndex = 0; currentActionIndex < panelActionsSelfCalib.length; currentActionIndex++) {
						if(!panelActionsSelfCalib[currentActionIndex].getSucceeded())
							break;
					}
				}


				if(rdbtn_RM_Corr.isSelected())
				{
					switch (currentActionIndex) {
					case 0:
						//Background measurement
						JOptionPane.showMessageDialog(frmWL_Calib,
								"Please inject light from NIR wide band light source.",
								"Wavenumber Calibration", JOptionPane.INFORMATION_MESSAGE);

						logger.info("WavelengthCalibrationBG Started");
						wavelengthCalibrationBG_Sequence();
						break;

					case 1:
						//Sample measurement
						JOptionPane.showMessageDialog(frmWL_Calib,
								"Please insert your sample.",
								"Wavenumber Calibration", JOptionPane.INFORMATION_MESSAGE);

						logger.info("WavelengthCalibration Started");
						wavelengthCalibrationSequence();

						break;

					default:
						JOptionPane.showMessageDialog(frmWL_Calib,"Unsupported Action!",
								"Wavelength & Wavenumber Correction", JOptionPane.OK_OPTION);
						break;
					}
				}
				else {
					switch (currentActionIndex) {
					case 0:
						//Self Correction

						JOptionPane.showMessageDialog(frmWL_Calib,
								"Please place a reference material with a flat spectral response at the front face of the light source.",
								"Self-Correction", JOptionPane.INFORMATION_MESSAGE);

						logger.info("Self_Correction_Factors_Calc Started");
						selfCorrCalibSequence();

						break;

					default:
						JOptionPane.showMessageDialog(frmWL_Calib,"Unsupported Action!",
								"Wavelength & Wavenumber Correction", JOptionPane.OK_OPTION);
						break;
					}
				}

			}
		});
		frmWL_Calib.getContentPane().add(btn_Proceed_WL_Calib, "cell 0 6 3 1,growx, alignx right");

		btn_Close_WL_Calib = new JButton("");
		btn_Close_WL_Calib.setFont(new Font("Dialog", Font.PLAIN, 12));
		btn_Close_WL_Calib.setText("Close");
		btn_Close_WL_Calib.setToolTipText("Close");
		btn_Close_WL_Calib.setMaximumSize(new Dimension((int)(p3Constants.MAX_WIDTH_OF_FIELD), btn_Close_WL_Calib.getPreferredSize().height));

		btn_Close_WL_Calib.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				onCloseOperation();
			}
		});
		frmWL_Calib.getContentPane().add(btn_Close_WL_Calib, "cell 0 6 3 1,growx, alignx right");
	}

	private void onCloseOperation()
	{
		UserInterface.stopEnablingButtons = false;
		// resume checking the status of the device
		UserInterface.checkDeviceStatusThreadStop = false;
		UserInterface.boardReadyRoutine(true);
		frmWL_Calib.dispose();
	}

	private static String getActionsString()
	{
		String result = "<html>";
		int checkCurrent = 0;

		if(rdbtn_RM_Corr.isSelected())
		{
			for(int i = 0; i < panelActionsRM.length; i++)
			{
				if(0 == checkCurrent)
				{
					if(panelActionsRM[i].getSucceeded())
					{
						result += "<font color='green'>" + panelActionsRM[i].getActionName() + " &#10004; </font><br />";
					}
					else
					{
						checkCurrent ++;
						result += "<font color='black'>" + panelActionsRM[i].getActionName() + "</font><br />";
					}
				}
				else
				{
					result += "<font color='gray'>" + panelActionsRM[i].getActionName() + "</font><br />";
				}
			}
		}
		else {
			for(int i = 0; i < panelActionsSelfCalib.length; i++)
			{
				if(0 == checkCurrent)
				{
					if(panelActionsSelfCalib[i].getSucceeded())
					{
						result += "<font color='green'>" + panelActionsSelfCalib[i].getActionName() + " &#10004; </font><br />";
					}
					else
					{
						checkCurrent ++;
						result += "<font color='black'>" + panelActionsSelfCalib[i].getActionName() + "</font><br />";
					}
				}
				else
				{
					result += "<font color='gray'>" + panelActionsSelfCalib[i].getActionName() + "</font><br />";
				}
			}
		}

		result += "</html>";
		return result;
	}


	private void wavelengthCalibrationBG_Sequence()
	{

		try
		{
			// stop checking the status of the device
			UserInterface.checkDeviceStatusThreadStop = true;

			try{

				SpectroscopyPanel.lastResolutionSelected = resolution;
				SpectroscopyPanel.lastZeroPaddingSelected = zeroPaddingMultiplier;
				SpectroscopyPanel.lastWindowSelected = apodizationIndex;
				InterSpecPanel.lastResolutionSelected = resolution;

				if(p3Constants.STANDARD_CALIBRATOR_DEFAULT_CHOISE.equals(cmb_StandardCalibrator_WL_Calib.getSelectedItem().toString())
						|| cmb_StandardCalibrator_WL_Calib.getSelectedItem().toString() == null
						|| cmb_StandardCalibrator_WL_Calib.getSelectedItem().toString() == "")
				{

					//conf. files changed, take background again
					JOptionPane.showMessageDialog(frmWL_Calib,
							"Wavelength calibration failed to start: Please select standard calibrator.",
							"Wavelength Calibration", JOptionPane.OK_OPTION);

					UserInterface.checkDeviceStatusThreadStop = false;
					return;
				}
			}
			catch(Exception ex)
			{
				JOptionPane.showMessageDialog(frmWL_Calib,
						"Wavelength calibration failed to start. Module setup failed!",
						"Wavelength Calibration", JOptionPane.OK_OPTION);

				UserInterface.checkDeviceStatusThreadStop = false;
				return;
			}

			UserInterface.progressTime = Double.parseDouble(runTime);
			UserInterface.progressPar.setValue(0);
			while(UserInterface.acquireDeviceToken() == false);
			p3AppManagerStatus status = UserInterface.applicationManager.wavelengthCalibrationBG(runTime, apodizationIndex, 
					zeroPaddingMultiplier, opticalSettings, commonWVN);
			

			if (p3AppManagerStatus.NO_ERROR != status) {
				if (p3AppManagerStatus.DEVICE_BUSY_ERROR != status) {
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;
				}

				JOptionPane.showMessageDialog(frmWL_Calib,
						"Wavelength calibration failed to start: " + UserInterface.convertErrorCodesToMessages(status),
						"Wavelength Calibration", JOptionPane.OK_OPTION);

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();

			} else {

				panelReadyRoutine(false);

				UserInterface.statusLabel.setText("Background measurement started. Please wait...");

				UserInterface.colorLabel.setForeground(Color.yellow);
				UserInterface.colorLabel.setBackground(Color.yellow);

			}
		}
		catch(Exception ex)
		{
			JOptionPane.showMessageDialog(frmWL_Calib,
					"Run failed to start. Please check inputs.",
					"Run NeoSpectra Micro", JOptionPane.OK_OPTION);

		}

		UserInterface.frmMain.revalidate();
		UserInterface.frmMain.repaint();
	}

	private void wavelengthCalibrationSequence()
	{

		try
		{
			// stop checking the status of the device
			UserInterface.checkDeviceStatusThreadStop = true;

			try{

				if(p3Constants.STANDARD_CALIBRATOR_DEFAULT_CHOISE.equals(cmb_StandardCalibrator_WL_Calib.getSelectedItem().toString())
						|| cmb_StandardCalibrator_WL_Calib.getSelectedItem().toString() == null
						|| cmb_StandardCalibrator_WL_Calib.getSelectedItem().toString() == "")
				{

					
					//conf. files changed, take background again
					JOptionPane.showMessageDialog(frmWL_Calib,
							"Wavelength calibration failed to start: Please select standard calibrator.",
							"Wavelength Calibration", JOptionPane.OK_OPTION);

					UserInterface.checkDeviceStatusThreadStop = false;
					return;
				}
			}
			catch(Exception ex)
			{
				JOptionPane.showMessageDialog(frmWL_Calib,
						"Wavelength calibration failed to start. Module setup failed!",
						"Wavelength Calibration", JOptionPane.OK_OPTION);

				UserInterface.checkDeviceStatusThreadStop = false;
				return;
			}

			UserInterface.progressTime = Double.parseDouble(runTime);
			UserInterface.progressPar.setValue(0);
			while(UserInterface.acquireDeviceToken() == false);
			p3AppManagerStatus status = UserInterface.applicationManager.wavelengthCalibration(runTime, 
					cmb_StandardCalibrator_WL_Calib.getSelectedItem().toString(),
					apodizationIndex,
					zeroPaddingMultiplier,
					opticalSettings,
					commonWVN);

			if (p3AppManagerStatus.NO_ERROR != status) {
				if (p3AppManagerStatus.DEVICE_BUSY_ERROR != status) {
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;
				}

				JOptionPane.showMessageDialog(frmWL_Calib,
						"Wavelength calibration failed to start: " + UserInterface.convertErrorCodesToMessages(status),
						"Wavelength Calibration", JOptionPane.OK_OPTION);
				

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();

			} else {

				panelReadyRoutine(false);

				UserInterface.statusLabel
				.setText("Wavelength calibration started. Please wait...");

				UserInterface.colorLabel.setForeground(Color.yellow);
				UserInterface.colorLabel.setBackground(Color.yellow);

			}
		}
		catch(Exception ex)
		{
			JOptionPane.showMessageDialog(frmWL_Calib,
					"Run failed to start. Please check inputs.",
					"Run NeoSpectra Micro", JOptionPane.OK_OPTION);

		}

		UserInterface.frmMain.revalidate();
		UserInterface.frmMain.repaint();
	}

	private void selfCorrCalibSequence()
	{
		logger.info("selfCorrCalibSequence function started");
		try{
			// stop checking the status of the device
			UserInterface.checkDeviceStatusThreadStop = true;

			try
			{
				logger.info("setSettings function (reg file rewrite) started");
				logger.info("setSettings function (reg file rewrite) finished");

				SpectroscopyPanel.lastResolutionSelected = p3Constants.TWO_POINTS_CORR_CALIB_FOLDER_NAME;
				SpectroscopyPanel.lastZeroPaddingSelected = zeroPaddingMultiplier;
				SpectroscopyPanel.lastWindowSelected = apodizationIndex;
				InterSpecPanel.lastResolutionSelected = p3Constants.TWO_POINTS_CORR_CALIB_FOLDER_NAME;

				logger.info("Setting Optical Settings started");
				logger.info("Setting Optical Settings finished");

			}
			catch(Exception ex)
			{
				JOptionPane.showMessageDialog(frmWL_Calib,
						"Run failed to start. Module setup failed!",
						"Self-Correction", JOptionPane.OK_OPTION);

				UserInterface.checkDeviceStatusThreadStop = false;
				return;
			}

			UserInterface.progressTime = Double.parseDouble(runTime);
			UserInterface.progressPar.setValue(0);
			logger.info("runInterSpec function started");
			while(UserInterface.acquireDeviceToken() == false);
			p3AppManagerStatus status = UserInterface.applicationManager.runCalibCorr(runTime, apodizationIndex, zeroPaddingMultiplier, opticalSettings, commonWVN);
			logger.info("runInterSpec function finished");
			if (p3AppManagerStatus.NO_ERROR != status) {

				if (p3AppManagerStatus.DEVICE_BUSY_ERROR != status) {
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;
				}

				JOptionPane.showMessageDialog(frmWL_Calib,
						"Run failed to start: " + UserInterface.convertErrorCodesToMessages(status),
						"Self-Correction", JOptionPane.OK_OPTION);

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();
			} else {
				logger.info("disabling GUI fields started");
				panelReadyRoutine(false);
				logger.info("disabling GUI fields finished");
				UserInterface.statusLabel.setText("Calibration started. Please wait...");

				UserInterface.colorLabel.setForeground(Color.yellow);
				UserInterface.colorLabel.setBackground(Color.yellow);

			}
		}
		catch(Exception ex)
		{
			JOptionPane.showMessageDialog(frmWL_Calib,
					"Run failed to start. Please check inputs.",
					"Self-Correction", JOptionPane.OK_OPTION);
		}
		UserInterface.frmMain.revalidate();
		UserInterface.frmMain.repaint();
	}


	public static void update(Object arg1) {
		if (arg1 instanceof p3AppManagerNotification) {
			logger.info("update method started");
			p3AppManagerNotification resp = (p3AppManagerNotification) arg1;

			switch (resp.getAction()) {
			case 25: // Interferogram gain and offset correction parameters calculation
				if (resp.getStatus() == 0) {

					panelActionsSelfCalib[0].setSucceeded(true);
					panelActionsSelfCalib[1].setSucceeded(true);

					lbl_Progress_Details_WL_Calib.setText(getActionsString());

					UserInterface.statusLabel.setText("Calibration is completed successfully!");
					UserInterface.colorLabel.setForeground(Color.green.darker());
					UserInterface.colorLabel.setBackground(Color.green.darker());

					UserInterface.progressTime = -1;
					UserInterface.progressPar.setValue(100);

					JOptionPane.showMessageDialog(frmWL_Calib,
							"Calibration Succeeded!",
							"Wavelength & Wavenumber Correction", JOptionPane.PLAIN_MESSAGE);

				} else {
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

					UserInterface.progressTime = -1;
					UserInterface.progressPar.setValue(100);

					JOptionPane.showMessageDialog(
							frmWL_Calib,
							UserInterface.convertErrorCodesToMessages(resp.getStatus()),
							"Wavelength & Wavenumber Correction", JOptionPane.OK_OPTION);
				}

				logger.info("switching actuation off started");
				logger.info("switching actuation off finished");

				logger.info("enabling panel GUI fields started");
				panelReadyRoutine(true);
				logger.info("enabling panel GUI fields finished");

				btn_Proceed_WL_Calib.setEnabled(false);

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();
				UserInterface.checkDeviceStatusThreadStop = false;

				break;
			case 26: // Wavelength calibration
				if (resp.getStatus() == 0) {

					panelActionsRM[1].setSucceeded(true);
					panelActionsRM[2].setSucceeded(true);

					lbl_Progress_Details_WL_Calib.setText(getActionsString());

					UserInterface.statusLabel.setText("Calibration is completed successfully!");
					UserInterface.colorLabel.setForeground(Color.green.darker());
					UserInterface.colorLabel.setBackground(Color.green.darker());

					UserInterface.progressTime = -1;
					UserInterface.progressPar.setValue(100);

					JOptionPane.showMessageDialog(frmWL_Calib,
							"Calibration Succeeded!",
							"Wavelength & Wavenumber Correction", JOptionPane.PLAIN_MESSAGE);
				} else {
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

					UserInterface.progressTime = -1;
					UserInterface.progressPar.setValue(100);

					JOptionPane.showMessageDialog(
							frmWL_Calib,
							UserInterface.convertErrorCodesToMessages(resp.getStatus()),
							"Wavelength & Wavenumber Correction", JOptionPane.OK_OPTION);
				}

				logger.info("switching actuation off started");
				logger.info("switching actuation off finished");

				logger.info("enabling panel GUI fields started");
				panelReadyRoutine(true);
				logger.info("enabling panel GUI fields finished");

				btn_Proceed_WL_Calib.setEnabled(false);

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();
				UserInterface.checkDeviceStatusThreadStop = false;

				break;
			case 30: // BG measurement for Wavelength calibration
				if (resp.getStatus() == 0) {

					panelActionsRM[0].setSucceeded(true);

					lbl_Progress_Details_WL_Calib.setText(getActionsString());

					UserInterface.statusLabel.setText("BG measurement is completed successfully!");
					UserInterface.colorLabel.setForeground(Color.green.darker());
					UserInterface.colorLabel.setBackground(Color.green.darker());

					UserInterface.progressTime = -1;
					UserInterface.progressPar.setValue(100);

				} else {
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

					UserInterface.progressTime = -1;
					UserInterface.progressPar.setValue(100);

					JOptionPane.showMessageDialog(
							frmWL_Calib,
							UserInterface.convertErrorCodesToMessages(resp.getStatus()),
							"Wavelength & Wavenumber Correction", JOptionPane.OK_OPTION);
				}

				logger.info("switching actuation off started");
				logger.info("switching actuation off finished");

				logger.info("enabling panel GUI fields started");
				panelReadyRoutine(true);
				logger.info("enabling panel GUI fields finished");

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();
				UserInterface.checkDeviceStatusThreadStop = false;

				break;
			default:
				// not supported action
				;
			}
		}

	}

	public static void panelReadyRoutine(boolean ready) {
		if (ready) {
			btn_Proceed_WL_Calib.setEnabled(true);
			btn_Close_WL_Calib.setEnabled(true);

		} else {
			btn_Proceed_WL_Calib.setEnabled(false);
			btn_Close_WL_Calib.setEnabled(false);

		}
	}

}