package sws.spectromost;

import java.awt.EventQueue;

import javax.imageio.ImageIO;
import javax.swing.JFrame;

import java.awt.Toolkit;

import javax.swing.JTabbedPane;

import java.awt.BorderLayout;
import java.awt.Desktop;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Image;

import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JCheckBoxMenuItem;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JMenu;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JRadioButton;
import javax.swing.SwingConstants;
import javax.swing.UIManager;
import javax.swing.border.BevelBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileFilter;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Observable;
import java.util.Observer;

import net.miginfocom.swing.MigLayout;

import org.apache.log4j.Logger;

import sws.p3AppManager_micro.p3AppManager_micro;
import sws.p3AppManager_micro.p3AppManagerImpl;
import sws.p3AppManager_micro.utils.p3AppManagerNotification;
import sws.p3AppManager_micro.utils.p3AppManagerUtils;
import sws.p3AppManager_micro.utils.p3Constants;
import sws.p3AppManager_micro.utils.p3Enumerations.p3AppManagerStatus;

import java.awt.Font;
import java.awt.Color;

public class UserInterface implements Observer {

	private static Logger logger = Logger.getLogger(UserInterface.class);

	static JFrame frmMain;

	/*
	 * ! boolean to indicate finishing switching device (upon notification)
	 */
	public static boolean switchingDeviceFinished = false;

	/*
	 * ! time to wait for switching device notification before time out
	 */
	public static final long switchingDeviceTimeOut = 2000;

	/*
	 * ! Path of the file you want to write values of ASIC registers to.
	 */
	public static String regFileToWrite = "";

	public static boolean stopEnablingButtons = false;

	/*
	 * ! state of actuation whether started or not
	 */
	public static boolean actState = false;

	/*
	 * ! boolean value to stop continuous sample when needed
	 */
	public static boolean isContinuousRun = false;
	public static boolean stopContinuousRun = false;

	/*
	 * ! gets the path of running application
	 */
	public static String APPLICATION_WORKING_DIRECTORY = p3Constants.Original_APPLICATION_WORKING_DIRECTORY;


	public static final String IMAGES_DIRECTORY = APPLICATION_WORKING_DIRECTORY + File.separatorChar + "Data" + File.separatorChar + "Images";
	public static final String LOGO_IMAGE_PATH = IMAGES_DIRECTORY + File.separatorChar + "NeoSpectraLogo.png";
	public static final String RUN_IMAGE_PATH = IMAGES_DIRECTORY + File.separatorChar + "run_new.png";
	public static final String RUN_BG_IMAGE_PATH = IMAGES_DIRECTORY + File.separatorChar + "run_bg.png";
	public static final String SAVE_IMAGE_PATH = IMAGES_DIRECTORY + File.separatorChar + "save.png";
	public static final String LOAD_IMAGE_PATH = IMAGES_DIRECTORY + File.separatorChar + "load.png";
	public static final String CAPTURE_IMAGE_PATH = IMAGES_DIRECTORY + File.separatorChar + "capture.png";
	public static final String DELETE_IMAGE_PATH = IMAGES_DIRECTORY + File.separatorChar + "delete.png";
	public static final String STOP_IMAGE_PATH = IMAGES_DIRECTORY + File.separatorChar + "stop_new.png";
	public static final String ICON_PATH = IMAGES_DIRECTORY + File.separatorChar + "Icon.png";

	public static final String GRAPH_FILES_FOLDER_PATH = APPLICATION_WORKING_DIRECTORY + File.separatorChar + "Temp_Graphs";

	public static final String OPTIONS_PATH = APPLICATION_WORKING_DIRECTORY + File.separatorChar + "Conf_Files";

	public static String defaultDirectorySaveLoad = "";

	public static String fileNameToSave = "";
	
	public static JCheckBoxMenuItem chb_Advanced_menuBar;
	
	public static JCheckBoxMenuItem chb_SleepAction_menuBar;
	
	public static JCheckBoxMenuItem chb_PowerOff_menuBar;
	/*
	 * ! Module ID to be written to the board EEPROM
	 */
	public static String moduleID = "";
	private static boolean DeviceToken = false;
	/*
	 * ! Buttons Image Scale.
	 */
	static final int SCALE = 25;
	static final int SMALL_SCALE = 20;

	static p3AppManager_micro applicationManager;

	static JLabel colorLabel;
	static JLabel statusLabel;
	static JProgressBar progressPar;

	static InterSpecPanel pnl_Inter_Spec;
	static SpectroscopyPanel pnl_Spectroscopy;

	private JTabbedPane tabbedPane;

	/*
	 * ! boolean value to stop checkDeviceStatusThread when needed
	 */
	static boolean checkDeviceStatusThreadStop = false;

	/*
	 * ! scan time of the current run to set the progress based on it.
	 */
	static double progressTime = -1;

	/*
	 * ! time of sleep of check device status thread
	 */
	static final int THREAD_SLEEP_TIME = 1000;

	/*
	 * ! boolean value to indicate whether the board was just restarted or not (used to force background reading after restarting board)
	 */
	static boolean boardRestarted = true;

	/*
	 * ! Size of Screen
	 */
	static Dimension screenSize;

	Thread checkDeviceStatusThread = new Thread() {

		public void run() {
			try {

				while (true) {
					
					Thread.sleep(THREAD_SLEEP_TIME);
					if (checkDeviceStatusThreadStop == false) {
						
						while(acquireDeviceToken() == false);
						p3AppManagerStatus status = applicationManager.checkDeviceStatus();
						releaseDeviceToken();
						

						if (p3AppManagerStatus.NO_ERROR == status) {

							if(!statusLabel.getText().equals("Background Measurement completed successfully!"))
							{
								statusLabel.setText("NeoSpectra Micro kit is Ready!");
								colorLabel.setForeground(Color.green.darker());
								colorLabel.setBackground(Color.green.darker());
								
								if(About.DVKVersion.equals(""))
								{
									while(acquireDeviceToken() == false);
									String[] versions = UserInterface.applicationManager.getSoftwareVersion();
									releaseDeviceToken();
									About.DVKVersion = versions[0];
									About.PiVersion = versions[1];
								}
							}

							if(!stopEnablingButtons)
							{
								boardReadyRoutine(true);
							}

						} else {

							progressPar.setValue(0);
							boardRestarted = true;

							if (p3AppManagerStatus.INITIALIZATION_IN_PROGRESS == status) {
								statusLabel.setText("Initializing NeoSpectra Micro kit. Please wait...");
								colorLabel.setForeground(Color.yellow);
								colorLabel.setBackground(Color.yellow);

							} else if (p3AppManagerStatus.BOARD_NOT_INITIALIZED_ERROR == status) {

								statusLabel.setText("NeoSpectra Micro kit is connected");
								colorLabel.setForeground(Color.yellow);
								colorLabel.setBackground(Color.yellow);

							} else if (p3AppManagerStatus.BOARD_DISTCONNECTED_ERROR == status) {

								statusLabel.setText("SpectroMOST Micro does not detect any connected NeoSpectra Micro kit");
								SpectroscopyPanel.lbl_moduleID_Spec.setText("");
								InterSpecPanel.lbl_ModuleID_Inter_Spec.setText("");

								SpectroscopyPanel.cmb_Optical_Settings_Spec.removeAllItems();
								InterSpecPanel.cmb_Optical_Settings_Inter_Spec.removeAllItems();

								InterSpecPanel.lastResolutionSelected = "";
								SpectroscopyPanel.lastResolutionSelected = "";
								SpectroscopyPanel.lastZeroPaddingSelected = "";
								SpectroscopyPanel.lastWindowSelected = "";
								SpectroscopyPanel.lastCommonWavenumberSelected = "";

								SpectroscopyPanel.lastOpticalSettingsSelected = "";
								InterSpecPanel.lastOpticalSettingsSelected = "";
								
								About.DVKVersion = "";
								About.PiVersion = "";

								colorLabel.setForeground(Color.gray);
								colorLabel.setBackground(Color.gray);

							} else {
								colorLabel.setForeground(Color.red);
								colorLabel.setBackground(Color.red);

								statusLabel.setText(convertErrorCodesToMessages(status) + ".");
							}

							boardReadyRoutine(false);

							InterSpecPanel.btn_SaveGraphs_Inter_Spec.setEnabled(true);
							InterSpecPanel.btn_LoadGraphs_Inter_Spec.setEnabled(true);
							InterSpecPanel.btn_ClearGraphs_Inter_Spec.setEnabled(true);
							
							SpectroscopyPanel.btn_SaveGraphs_Spec.setEnabled(true);
							SpectroscopyPanel.btn_LoadGraphs_Spec.setEnabled(true);
							SpectroscopyPanel.btn_ClearGraphs_Spec.setEnabled(true);
							
							SpectroscopyPanel.btn_Stop_Spec.setEnabled(false);
							InterSpecPanel.btn_Stop_Inter_Spec.setEnabled(false);

						}
						frmMain.revalidate();
						frmMain.repaint();
					}
					else
					{
						if(progressTime != -1)
						{
							if((int)((THREAD_SLEEP_TIME/progressTime) * 100) != 0)
							{
								progressPar.setValue(progressPar.getValue() + (int)((THREAD_SLEEP_TIME/progressTime) * 100));
							}
							else
							{
								progressPar.setValue(progressPar.getValue() + (int)Math.ceil((THREAD_SLEEP_TIME/progressTime) * 100));	
							}
						}
					}
				}
			} catch (Exception ex) {
				logger.error(ex.getMessage());
			}
		}
	};

	public static synchronized boolean acquireDeviceToken(){
		if(DeviceToken == true){
			return false;
		}else{
			DeviceToken = true;
			return true;
		}
	}
	
	public static synchronized void releaseDeviceToken(){
		DeviceToken = false;
	}
	/*
	 * ! Launch the application.
	 */
	public static void main(String[] args) {

		EventQueue.invokeLater(new Runnable() {

			public void run() {
				try {
					logger.info("Starting Application.....");
					
					try {
						UIManager.setLookAndFeel("com.sun.java.swing.plaf.nimbus.NimbusLookAndFeel");
					} catch (Exception e1) {
						e1.printStackTrace();
					}
					
					UserInterface window = new UserInterface();
					UserInterface.frmMain.setVisible(true);

					// Initialize the Application Manager
					p3AppManagerStatus status = applicationManager.initializeCore();

					if (p3AppManagerStatus.NO_ERROR != status) {
						colorLabel.setForeground(Color.red);
						colorLabel.setBackground(Color.red);

						logger.error("error happen while trying to run current initialize \n"
								+ convertErrorCodesToMessages(status));
					}
					//checkDeviceStatusThreadStop = true;
					window.checkDeviceStatusThread.start();

				} catch (Exception e) {
					logger.error(e.getMessage());
				}
			}
		});
	}

	/*
	 * ! Create the application.
	 */
	public UserInterface() {
		initialize();

		// Construct the Application Manager
		applicationManager = new p3AppManagerImpl();
		applicationManager.addObserver(this);
	}

	/*
	 * ! Initialize the contents of the frame.
	 */
	private void initialize() {

		screenSize = Toolkit.getDefaultToolkit().getScreenSize();

		frmMain = new JFrame();
		try {
			File imgFile = new File(ICON_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				frmMain.setIconImage(img);
			} else {
				JOptionPane.showMessageDialog(null, "Icon doesn't exist.",
						"Loading image", JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(null,
					"Error loading Icon:" + ex.getMessage(), "Loading icon",
					JOptionPane.OK_OPTION);
		}

		p3AppManagerUtils.removeDir(p3Constants.getPath(GRAPH_FILES_FOLDER_PATH));
		p3AppManagerUtils.createDir(p3Constants.getPath(GRAPH_FILES_FOLDER_PATH));

		frmMain.setBackground(new Color(176, 196, 222));
		frmMain.getContentPane().setBackground(new Color(176, 196, 222));
		frmMain.setVisible(true);
		frmMain.setName("frame1");
		frmMain.setMinimumSize(new Dimension(screenSize.width - 70, screenSize.height - 70));
		frmMain.setMaximumSize(new Dimension(screenSize.width - 70, screenSize.height - 70));
		frmMain.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent arg0) {
				UserInterface_closeForm();
			}
		});

		frmMain.addComponentListener(new ComponentListener() {
			int state = 0;
			@Override
			public void componentShown(ComponentEvent arg0) {
			}

			@Override
			public void componentResized(ComponentEvent arg0) {
				if(state==1)
				{
					return;
				}
				state++;
			}

			@Override
			public void componentMoved(ComponentEvent arg0) {
				state=0;
			}

			@Override
			public void componentHidden(ComponentEvent arg0) {
				System.out.println("--------------------------------------------------------------------------");
			}
		});
		frmMain.setTitle("SpectroMOST Micro\u2122");
		frmMain.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		frmMain.setResizable(true);
		frmMain.setExtendedState(Frame.MAXIMIZED_BOTH);

		JMenuBar menuBar = new JMenuBar();
		frmMain.getContentPane().add(menuBar, BorderLayout.NORTH);

		JMenu menu = new JMenu("Spectrometer");
		menuBar.add(menu);

		chb_Advanced_menuBar = new JCheckBoxMenuItem("Advanced / FFT Settings", false);
		chb_Advanced_menuBar.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				ShowAdvancedSettings();
			}

		});
		menu.add(chb_Advanced_menuBar);
		
		chb_SleepAction_menuBar = new JCheckBoxMenuItem("Sleep", false);
		chb_SleepAction_menuBar.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				runSleepAction();
			}

		});
		menu.add(chb_SleepAction_menuBar);
		
		chb_PowerOff_menuBar = new JCheckBoxMenuItem("Power off", false);
		chb_PowerOff_menuBar.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				runPowerOff();
			}

		});
		menu.add(chb_PowerOff_menuBar);
		
		JMenuItem mntmExit = new JMenuItem("Exit                   ");
		mntmExit.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				UserInterface_closeForm();
			}

		});
		menu.add(mntmExit);

		JMenu mnHelp = new JMenu("Help");
		menuBar.add(mnHelp);

		JMenuItem mntmUerManual = new JMenuItem("User Manual");
		mntmUerManual.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				if (Desktop.isDesktopSupported()) {
				    try {
				        File userManual = new File(APPLICATION_WORKING_DIRECTORY + File.separatorChar + "Data" + File.separatorChar + "User Manual.pdf");
				        Desktop.getDesktop().open(userManual);
				    } catch (IllegalArgumentException ex) {
				    	JOptionPane.showMessageDialog(null,
								"Unable to find the User Manual.\r\nFor more help, please contact Si-Ware Systems.",
								"Failed to open User Manual", JOptionPane.OK_OPTION);
				    } catch (FileNotFoundException ex) {
				    	JOptionPane.showMessageDialog(null,
								"Unable to find the User Manual.\r\nFor more help, please contact Si-Ware Systems.",
								"Failed to open User Manual", JOptionPane.OK_OPTION);
				    } catch (IOException ex) {
				    	JOptionPane.showMessageDialog(null,
								"No application registered to view a PDF file.",
								"Failed to open User Manual", JOptionPane.OK_OPTION);
				    } catch (Exception ex) {
				    	JOptionPane.showMessageDialog(null,
								"Unable to open the User Manual.\r\nFor more help, please contact Si-Ware Systems.",
								"Failed to open User Manual", JOptionPane.OK_OPTION);
				    } 
				}else{
					JOptionPane.showMessageDialog(null,
							"Unable to open the User Manual.\r\nFor more help, please contact Si-Ware Systems.",
							"Failed to open User Manual", JOptionPane.OK_OPTION);
				}
			}
		});
		mnHelp.add(mntmUerManual);
		
		JMenuItem mntmAbout = new JMenuItem("About");
		mntmAbout.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				About aboutDialog = new About();
				aboutDialog.setDefaultCloseOperation(JDialog.DISPOSE_ON_CLOSE);
				aboutDialog.setVisible(true);
			}
		});
		mnHelp.add(mntmAbout);
		
		tabbedPane = new JTabbedPane(JTabbedPane.TOP);
		tabbedPane.setFont(new Font("DejaVu Serif Condensed", Font.BOLD, 13));
		tabbedPane.addChangeListener(new ChangeListener() {
			public void stateChanged(ChangeEvent arg0) {
				try{
					if(tabbedPane.getSelectedIndex() == 1)
					{
						InterSpecPanel.txt_RunTime_Inter_Spec.setText(SpectroscopyPanel.txt_RunTime_Spec.getText());
						InterSpecPanel.rdbtn_SingleRun_Inter_Spec.setSelected(SpectroscopyPanel.rdbtn_SingleRun_Spec.isSelected());
						InterSpecPanel.rdbtn_ContRun_Inter_Spec.setSelected(SpectroscopyPanel.rdbtn_ContRun_Spec.isSelected());
						InterSpecPanel.btn_Capture_Interfero.setEnabled(SpectroscopyPanel.btn_Capture_Spec.isEnabled());
					}
					else if(tabbedPane.getSelectedIndex() == 0)
					{
						SpectroscopyPanel.txt_RunTime_Spec.setText(InterSpecPanel.txt_RunTime_Inter_Spec.getText());
						SpectroscopyPanel.rdbtn_SingleRun_Spec.setSelected(InterSpecPanel.rdbtn_SingleRun_Inter_Spec.isSelected());
						SpectroscopyPanel.rdbtn_ContRun_Spec.setSelected(InterSpecPanel.rdbtn_ContRun_Inter_Spec.isSelected());
						SpectroscopyPanel.btn_Capture_Spec.setEnabled(InterSpecPanel.btn_Capture_Interfero.isEnabled());
					}
				}
				catch(Exception e)
				{}
			}
		});
		frmMain.getContentPane().add(tabbedPane, BorderLayout.CENTER);


		
		// create the status bar panel and show it down the bottom of the frame
		JPanel statusPanel = new JPanel();
		statusPanel.setBorder(new BevelBorder(BevelBorder.LOWERED));
		UserInterface.frmMain.getContentPane().add(statusPanel, BorderLayout.SOUTH);
		statusPanel.setPreferredSize(new Dimension(UserInterface.frmMain .getWidth(), 20));
		statusPanel.setLayout(new BoxLayout(statusPanel, BoxLayout.X_AXIS));

		JLabel lbl_progress = new JLabel(" Data transfer progress");
		lbl_progress.setHorizontalAlignment(SwingConstants.LEFT);
		statusPanel.add(lbl_progress);

		UserInterface.progressPar = new JProgressBar(0);
		UserInterface.progressPar.setAlignmentX(SwingConstants.LEFT);
		UserInterface.progressPar.setMaximumSize(new Dimension(2 * p3Constants.MAX_WIDTH_OF_FIELD, (int)UserInterface.progressPar.getPreferredSize().getHeight()));
		UserInterface.progressPar.setOpaque(true);
		statusPanel.add(UserInterface.progressPar);

		JLabel lbl_status = new JLabel("  Status");
		lbl_status.setHorizontalAlignment(SwingConstants.LEFT);
		statusPanel.add(lbl_status);

		UserInterface.colorLabel = new JLabel("                                 ");
		UserInterface.colorLabel.setHorizontalAlignment(SwingConstants.LEFT);
		UserInterface.colorLabel.setVerticalAlignment(SwingConstants.CENTER);
		UserInterface.colorLabel.setMaximumSize(new Dimension(2 * p3Constants.MAX_WIDTH_OF_FIELD, (int)(UserInterface.colorLabel.getPreferredSize().getHeight() + 2)));
		UserInterface.colorLabel.setOpaque(true);
		UserInterface.colorLabel.setForeground(Color.gray);
		UserInterface.colorLabel.setBackground(Color.gray);
		statusPanel.add(UserInterface.colorLabel);

		UserInterface.statusLabel = new JLabel("");
		UserInterface.statusLabel.setHorizontalAlignment(SwingConstants.LEFT);
		statusPanel.add(UserInterface.statusLabel);

		pnl_Spectroscopy = new SpectroscopyPanel();
		tabbedPane.addTab("Spectrum", null, pnl_Spectroscopy, null);

		pnl_Inter_Spec = new InterSpecPanel();
		tabbedPane.addTab("PSD", null, pnl_Inter_Spec, null);
	}

	private void UserInterface_closeForm() {
		int dialogButton = JOptionPane.YES_NO_OPTION;
		int dialogResult = JOptionPane.showConfirmDialog(null, "Are you sure you want to exit?", "Exit!", dialogButton);
		if (dialogResult == JOptionPane.YES_OPTION) {

			//Remove all temp saved plots
			p3AppManagerUtils.removeDir(p3Constants.getPath(GRAPH_FILES_FOLDER_PATH));

			//Exit
			System.exit(0);
		}
	}

	@Override
	public void update(Observable arg0, Object arg1) {
		if (arg1 instanceof p3AppManagerNotification) {
			p3AppManagerNotification resp = (p3AppManagerNotification) arg1;
			
			switch (resp.getAction()) {

			case 0: // Initialization
				if (resp.getStatus() == 0) {
					statusLabel.setText("Initialization of NeoSpectra Micro kit completed successfully!");
					initializationPostAction();
					//checkDeviceStatusThreadStop = false;
				} else {
					colorLabel.setForeground(Color.red);
					colorLabel.setBackground(Color.red);

					statusLabel.setText(convertErrorCodesToMessages(resp.getStatus()));
				}
				break;

			case 1: // Interferogram & Spectrum Run
				pnl_Inter_Spec.update(arg1);
				break;
			case 2: // Spectroscopy background run
				pnl_Spectroscopy.update(arg1);
				
				break;
			case 3: // Spectroscopy sample Run
				pnl_Spectroscopy.update(arg1);
				break;
			case 4: // actuation setting
				if (resp.getStatus() != 0) {
					colorLabel.setForeground(Color.red);
					colorLabel.setBackground(Color.red);

					JOptionPane.showMessageDialog(
							null,
							convertErrorCodesToMessages(resp.getStatus()), "switch device",
							JOptionPane.OK_OPTION);

				}
				else
				{
					switchingDeviceFinished = true;
				}
				releaseDeviceToken();
				checkDeviceStatusThreadStop = false;
				break;

			case 23: // Adaptive Gain
				try {

					UserInterface.progressTime = -1;
					UserInterface.progressPar.setValue(100);

					if (resp.getStatus() == 0) {
						UserInterface.statusLabel.setText("Gain Adjusment completed successfully.");
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
					releaseDeviceToken();
					UserInterface.checkDeviceStatusThreadStop = false;

					UserInterface.boardReadyRoutine(true);
					
				} catch (Exception e) {
					colorLabel.setForeground(Color.red);
					colorLabel.setBackground(Color.red);

					progressTime = -1;
					progressPar.setValue(100);

					JOptionPane.showMessageDialog(null,
							"Failed to save gain settings.",
							"Gain Adjustment", JOptionPane.OK_OPTION);

					logger.info("enabling GUI fields started");
					boardReadyRoutine(true);
					logger.info("enabling GUI fields finished");

					// resume checking the status of the device
					releaseDeviceToken();
					checkDeviceStatusThreadStop = false;	
				}

				break;
			case 25: //Interferogram gain and offset correction parameters calculation

				try {
					WL_WN_CalibrationPanel.update(arg1);
				} catch (Exception e) {
					colorLabel.setForeground(Color.red);
					colorLabel.setBackground(Color.red);

					progressTime = -1;
					progressPar.setValue(100);

					JOptionPane.showMessageDialog(
							null,
							convertErrorCodesToMessages(resp.getStatus()),
							"Self Calibration", JOptionPane.OK_OPTION);

					logger.info("switching actuation off started");

					logger.info("switching actuation off finished");

					logger.info("enabling GUI fields started");
					boardReadyRoutine(true);
					logger.info("enabling GUI fields finished");

					// resume checking the status of the device
					releaseDeviceToken();
					checkDeviceStatusThreadStop = false;
				}
				break;
			case 26: //Wavelength calibration in spectrum tab

				try {
					WL_WN_CalibrationPanel.update(arg1);
				} catch (Exception e) {
					colorLabel.setForeground(Color.red);
					colorLabel.setBackground(Color.red);

					progressTime = -1;
					progressPar.setValue(100);

					JOptionPane.showMessageDialog(
							null,
							convertErrorCodesToMessages(resp.getStatus()),
							"Wavenumber Calibration", JOptionPane.OK_OPTION);

					logger.info("switching actuation off started");

					logger.info("switching actuation off finished");

					logger.info("enabling GUI fields started");
					boardReadyRoutine(true);
					logger.info("enabling GUI fields finished");

					// resume checking the status of the device
					releaseDeviceToken();
					checkDeviceStatusThreadStop = false;
				}

				break;
			case 28: // Gain Adjustment Spec BG
				try {

					UserInterface.progressTime = -1;
					UserInterface.progressPar.setValue(100);

					if (resp.getStatus() == 0) {
						UserInterface.statusLabel.setText("Gain Adjusment completed successfully.");
						UserInterface.colorLabel.setForeground(Color.green.darker());
						UserInterface.colorLabel.setBackground(Color.green.darker());
						UserInterface.boardRestarted = false;
						
						double[][] data = UserInterface.applicationManager.getGainAdjustSpecData();
						
						SaveOpticalGainPanel savePanel = new SaveOpticalGainPanel((int)data[0][0]);
						savePanel.frmSaveOpticalGain.setVisible(true);

					} else {

						UserInterface.colorLabel.setForeground(Color.red);
						UserInterface.colorLabel.setBackground(Color.red);

						JOptionPane.showMessageDialog(
								null,
								UserInterface.convertErrorCodesToMessages(resp.getStatus()), "Run Gain Adjusment",
								JOptionPane.OK_OPTION);

						// resume checking the status of the device
						releaseDeviceToken();
						UserInterface.checkDeviceStatusThreadStop = false;


						UserInterface.boardReadyRoutine(true);
					}
					
				} catch (Exception e) {
					colorLabel.setForeground(Color.red);
					colorLabel.setBackground(Color.red);

					progressTime = -1;
					progressPar.setValue(100);

					JOptionPane.showMessageDialog(null,
							"Failed to save gain settings.",
							"Gain Adjustment", JOptionPane.OK_OPTION);

					logger.info("enabling GUI fields started");
					boardReadyRoutine(true);
					logger.info("enabling GUI fields finished");

					// resume checking the status of the device
					releaseDeviceToken();
					checkDeviceStatusThreadStop = false;	
				}

				break;
			case 30: //BG measurement for Wavelength calibration in spectrum tab

				try {
					WL_WN_CalibrationPanel.update(arg1);
				} catch (Exception e) {
					colorLabel.setForeground(Color.red);
					colorLabel.setBackground(Color.red);

					progressTime = -1;
					progressPar.setValue(100);

					JOptionPane.showMessageDialog(
							null,
							convertErrorCodesToMessages(resp.getStatus()),
							"Wavenumber Calibration", JOptionPane.OK_OPTION);

					logger.info("switching actuation off started");

					logger.info("switching actuation off finished");

					logger.info("enabling GUI fields started");
					boardReadyRoutine(true);
					logger.info("enabling GUI fields finished");

					// resume checking the status of the device
					releaseDeviceToken();
					checkDeviceStatusThreadStop = false;
				}

				break;
			case 33:// Restore default settings
				if (resp.getStatus() == 0) {
					UserInterface.statusLabel.setText("Restoring to default completed successfully!");

					UserInterface.colorLabel.setForeground(Color.green.darker());
					UserInterface.colorLabel.setBackground(Color.green.darker());


				} else {
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

					JOptionPane.showMessageDialog(
							null,
							"Restore default settings has error: "
									+ resp.getStatus(), "Restore Default Settings",
									JOptionPane.OK_OPTION);
				}
				UserInterface.boardReadyRoutine(true);


				UserInterface.displayInterSpecOpticalSettings();
				UserInterface.displaySpecOpticalSettings();
				
				// resume checking the status of the device
				releaseDeviceToken();
				UserInterface.checkDeviceStatusThreadStop = false;
				break;
			case 34:// burn settings
				if (resp.getStatus() == 0) {
					
					UserInterface.statusLabel.setText("Burn Setting completed successfully!");

					UserInterface.colorLabel.setForeground(Color.green.darker());
					UserInterface.colorLabel.setBackground(Color.green.darker());

				} else {
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

					JOptionPane.showMessageDialog(
							null,
							"Burn settings has error: "
									+ resp.getStatus(), "Burn settings",
									JOptionPane.OK_OPTION);
				}
				UserInterface.boardReadyRoutine(true);
				// resume checking the status of the device
				releaseDeviceToken();
				UserInterface.checkDeviceStatusThreadStop = false;
				break;
			case 36:// Sleep Action
				if(resp.getStatus() == 0){
					UserInterface.statusLabel.setText("Sleep Action completed successfully!");


					UserInterface.chb_SleepAction_menuBar.setEnabled(true);
					
					UserInterface.colorLabel.setForeground(Color.blue.darker());
					UserInterface.colorLabel.setBackground(Color.blue.darker());
				}else{
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

					UserInterface.boardReadyRoutine(true);
					
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;
					
					UserInterface.chb_SleepAction_menuBar.setSelected(false);
					
					JOptionPane.showMessageDialog(
							null,
							"Sleep action has error: "
									+ resp.getStatus(), "Sleep action",
									JOptionPane.OK_OPTION);
				}
				
				releaseDeviceToken();
				break;
			case 37:// Wake up Action
				if(resp.getStatus() == 0){
					UserInterface.statusLabel.setText("Wake up Action completed successfully!");

					UserInterface.colorLabel.setForeground(Color.green.darker());
					UserInterface.colorLabel.setBackground(Color.green.darker());
					UserInterface.boardReadyRoutine(true);
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;
				}else{
					UserInterface.statusLabel.setText("Wake up Action failed!");
					
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

					UserInterface.chb_SleepAction_menuBar.setEnabled(true);
					UserInterface.chb_SleepAction_menuBar.setSelected(true);
					
					JOptionPane.showMessageDialog(
							null,
							"Wake up action has error: "
									+ resp.getStatus(), "Wake up action",
									JOptionPane.OK_OPTION);
				}
				releaseDeviceToken();
				
				break;
			case 38:// Power off
				if(resp.getStatus() == 0){
					UserInterface.statusLabel.setText("Power Off completed successfully!");


					UserInterface.chb_PowerOff_menuBar.setEnabled(true);
					
					UserInterface.colorLabel.setForeground(Color.blue.darker());
					UserInterface.colorLabel.setBackground(Color.blue.darker());
				}else{
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

					UserInterface.boardReadyRoutine(true);
					
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;
					
					UserInterface.chb_PowerOff_menuBar.setSelected(false);
					
					JOptionPane.showMessageDialog(
							null,
							"Power off has error: "
									+ resp.getStatus(), "Power off",
									JOptionPane.OK_OPTION);
				}
				
				releaseDeviceToken();
				break;
			case 39:// Power on
				if(resp.getStatus() == 0){
					UserInterface.statusLabel.setText("Power on completed successfully!");

					UserInterface.colorLabel.setForeground(Color.green.darker());
					UserInterface.colorLabel.setBackground(Color.green.darker());
					UserInterface.boardReadyRoutine(true);
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;
				}else{
					UserInterface.statusLabel.setText("Power on failed!");
					
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

					UserInterface.chb_PowerOff_menuBar.setEnabled(true);
					UserInterface.chb_PowerOff_menuBar.setSelected(true);
					
					JOptionPane.showMessageDialog(
							null,
							"Power on has error: "
									+ resp.getStatus(), "Power on",
									JOptionPane.OK_OPTION);
				}
				releaseDeviceToken();
				
				break;
			default:
				// not supported action
				;
			}
			frmMain.revalidate();
			frmMain.repaint();
		}
	}

	public static void initializationPostAction() {
		try
		{
			moduleID = applicationManager.getDeviceId();
			SpectroscopyPanel.lbl_moduleID_Spec.setText(moduleID);
			InterSpecPanel.lbl_ModuleID_Inter_Spec.setText(moduleID);
			
			String opticalFilePath = p3AppManagerUtils.formatString(p3Constants.getPath(p3Constants.OPTICAL_SETTINGS_FILE_PATH), UserInterface.moduleID);
			File opticalFile = new File(opticalFilePath);
			
			String modulePath = p3Constants.Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + "Conf_Files" + File.separatorChar + moduleID;
			File module = new File(modulePath);
			
			String confFilesPath = p3Constants.Original_APPLICATION_WORKING_DIRECTORY + File.separatorChar + "Conf_Files";
			File confFiles = new File(confFilesPath);
			
			if(confFiles.exists()){
				
				if(module.exists()){
					
					if(!opticalFile.exists()){
						opticalFile.createNewFile();
					}
				}else{
					module.mkdir();
					opticalFile.createNewFile();
				}
			}else{
				confFiles.mkdir();
				module.mkdir();
				opticalFile.createNewFile();
			}
			
			displayInterSpecOpticalSettings();
			displaySpecOpticalSettings();

			displayStandardCalibrators();

			if(SpectroscopyPanel.cmb_Optical_Settings_Spec.getItemCount() > 0)
				SpectroscopyPanel.cmb_Optical_Settings_Spec.setSelectedIndex(0);

			if(InterSpecPanel.cmb_Optical_Settings_Inter_Spec.getItemCount() > 0)
				InterSpecPanel.cmb_Optical_Settings_Inter_Spec.setSelectedIndex(0);
					

			SpectroscopyPanel.cmb_Apodization_Spec.setSelectedIndex(p3Constants.apodizationDefaultIndex);
			InterSpecPanel.cmb_Apodization_Inter_Spec.setSelectedIndex(p3Constants.apodizationDefaultIndex);

			SpectroscopyPanel.cmb_ZeroPadding_Spec.setSelectedIndex(p3Constants.paddingDefaultIndex);
			InterSpecPanel.cmb_ZeroPadding_Inter_Spec.setSelectedIndex(p3Constants.paddingDefaultIndex);
		}
		catch(Exception c)
		{
			logger.error(c.getMessage());
		}
		colorLabel.setForeground(Color.green.darker());
		colorLabel.setBackground(Color.green.darker());
	}

	public static String getOpticalGainValue(String item){
		try {

			String path = p3AppManagerUtils.formatString(p3Constants.getPath(p3Constants.OPTICAL_SETTINGS_FILE_PATH), UserInterface.moduleID);

			File filePath = new File(path);

			if(filePath.exists())
			{
				BufferedReader reader = null;
				try {
					reader = new BufferedReader(new FileReader(path));

					String line = null;
					while ((line = reader.readLine()) != null) {
						if(line.contains(":"))
						{
							if(line.split(":")[0].equals(item))
							{
								return line.split(":")[1];
							}
						}
					}

				} catch (Exception ex) {
					return null;
				} finally {
					try {
						reader.close();
					} catch (Exception ex) {
						return null;
					}
				}
			}

			return "-1";


		} catch (Exception e) {
			logger.error(e.getMessage());
		}
		return null;
	}
	
	public static int saveOpticalGainRoutine(String name, int value)
	{
		try {

			String path = p3AppManagerUtils.formatString(p3Constants.getPath(p3Constants.OPTICAL_SETTINGS_FILE_PATH), UserInterface.moduleID);
			ArrayList<String> list = new ArrayList<String>();

			File filePath = new File(path);

			if(filePath.exists())
			{
				BufferedReader reader = null;
				try {
					reader = new BufferedReader(new FileReader(path));

					String line = null;
					while ((line = reader.readLine()) != null) {
						if(line.split(":")[0].trim().equals(name))
						{
							return 1;
						}
							list.add(line);
					}

				} catch (Exception ex) {
					return -1;
				} finally {
					try {
						reader.close();
					} catch (Exception ex) {
						return -1;
					}
				}

				BufferedWriter writer= null;
				try {
					writer = new BufferedWriter(new FileWriter(path, false));
					for(String line : list)
					{
						writer.write(line + "\r\n");
					}
					writer.write(name + ":" + Integer.toString(value) + "\r\n");
				} catch (Exception ex) {
					return -1;
				} finally {
					try {
						writer.close();
					} catch (Exception ex) {
						return -1;
					}
				}
			}

		return 0;	

		} catch (Exception e) {
			logger.error(e.getMessage());
			return -1;
		}
	}
	
	public static void updateSelectedOpticalSetting(){
		InterSpecPanel.cmb_Optical_Settings_Inter_Spec.setSelectedIndex(InterSpecPanel.cmb_Optical_Settings_Inter_Spec.getItemCount() - 1);
		SpectroscopyPanel.cmb_Optical_Settings_Spec.setSelectedIndex(SpectroscopyPanel.cmb_Optical_Settings_Spec.getItemCount() - 1);
	}
	
	public static boolean writeGraphFile(double[] x, double[] y, String path, String header) {

		Writer writer = null;

		try {

			writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(path), "utf-8"));
			writer.write(header);
			writer.write("\n");
			int length = x.length < y.length ? x.length : y.length;

			for (int i = 0; i < length; i++) {
				writer.write(Double.toString(x[i]) + "\t" + Double.toString(y[i]));
				writer.write("\n");
			}

			return true;

		} catch (IOException ex) {
			logger.error(ex.getMessage());
			return false;
		} finally {
			try {
				writer.close();
			} catch (Exception ex) {
				logger.error(ex.getMessage());
			}
		}
	}

	static double[] switch_NM_CM(double[] data) {
		double[] xAxis = new double[data.length];
		for (int i = 0; i < xAxis.length; i++) {
			xAxis[i] = 10000000 / data[i];
		}
		return xAxis;
	}

	static double[][] loadGraphDataFromFile(String strFilePath) {
		ArrayList<Double> xList = new ArrayList<Double>();
		ArrayList<Double> yList = new ArrayList<Double>();
		BufferedReader reader = null;
		try {
			reader = new BufferedReader(new FileReader(strFilePath));

			// Read units line (do nothing with it for now)
			String line = reader.readLine();
			if (line.contains("y_Axis:Absorbance")) {

				SpectroscopyPanel.rdbtn_Absorbance.setSelected(true);
				SpectroscopyPanel.rdbtn_Refl_Trans.setSelected(false);
			} else if (line.contains("y_Axis:%Transmittance") || line.contains("y_Axis:%Reflectance")) {
				SpectroscopyPanel.rdbtn_Refl_Trans.setSelected(true);
				SpectroscopyPanel.rdbtn_Absorbance.setSelected(false);
			}
			if (line.contains("x_Axis:Wavenumber (cm-1)")) {

				SpectroscopyPanel.rdbtn_cm_Spec.setSelected(true);
				SpectroscopyPanel.rdbtn_nm_Spec.setSelected(false);

				InterSpecPanel.rdbtn_cm_Inter_Spec.setSelected(true);
				InterSpecPanel.rdbtn_nm_Inter_Spec.setSelected(false);
			} else if (line.contains("x_Axis:Wavelength (nm)")) {

				SpectroscopyPanel.rdbtn_nm_Spec.setSelected(true);
				SpectroscopyPanel.rdbtn_nm_Spec.setSelected(false);

				InterSpecPanel.rdbtn_nm_Inter_Spec.setSelected(true);
				InterSpecPanel.rdbtn_cm_Inter_Spec.setSelected(false);
			}
			while ((line = reader.readLine()) != null) {
				String[] strLineTokens = line.split("\t");

				if (strLineTokens.length == 2) {
					xList.add(Double.parseDouble(strLineTokens[0]));
					yList.add(Double.parseDouble(strLineTokens[1]));
				}
			}

		} catch (Exception ex) {
			logger.error(ex.getMessage());
			JOptionPane.showMessageDialog(null, ex.getMessage(), "Load error", JOptionPane.ERROR_MESSAGE);
			return null;
		} finally {
			try {
				reader.close();
			} catch (Exception ex) {
				logger.error(ex.getMessage());
			}
		}

		double[][] arrayToReturn = new double[2][];
		arrayToReturn[0] = new double[xList.size()];
		arrayToReturn[1] = new double[yList.size()];

		for (int y = 0; y < arrayToReturn[0].length; y++) {
			arrayToReturn[0][y] = xList.get(y);
		}

		for (int y = 0; y < arrayToReturn[1].length; y++) {
			arrayToReturn[1][y] = yList.get(y);
		}

		xList = null;
		yList = null;

		return arrayToReturn;
	}

	public static boolean checkEmptyDirectory(String filePath) {

		File file = new File(filePath);

		if (file.isDirectory()) {
			if (file.list().length > 0) {
				return false;
			} else {
				return true;
			}
		}
		return false;
	}

	/*
	 * !converts transmission data to absorbance data
	 */
	public static double[] convertRefltoAbs(double[] data) {
		double[] xInverse = new double[data.length];
		for (int i = 0; i < xInverse.length; i++) {
			xInverse[i] = -(Math.log10(data[i] / 100.0));
		}
		return xInverse;
	}

	/*
	 * !converts absorbance data to transmission data
	 */
	public static double[] convertAbstoRefl(double[] data) {
		double[] xInverse = new double[data.length];
		for (int i = 0; i < xInverse.length; i++) {
			xInverse[i] = 100.0 * Math.pow(10.0, -data[i]);
		}
		return xInverse;
	}

	public static double[] convertDataToT(double[] data) {
		double[] TArray = new double[data.length];
		for (int i = 0; i < TArray.length; i++) {
			TArray[i] = data[i] * 100;
		}
		return TArray;
	}

	public static String[] readOptions() {
		String[] temps = null;
		String[] options = null;
		try {
			if(!moduleID.equals(""))
			{
				String directory = p3Constants.getPath(OPTIONS_PATH) + File.separatorChar + moduleID;
				File dir = new File(directory);

				// lists all folders in module ID (temperature folders).
				File[] subDirs = dir.listFiles(new FileFilter() {
					public boolean accept(File pathname) {
						return pathname.isDirectory() && (!pathname.getName().equals(".svn"));
					}
				});
				temps = new String[subDirs.length];
				for (int i = 0; i< subDirs.length; i++) {
					temps[i] = subDirs[i].getName();
				}
				directory += File.separatorChar + temps[0];
				dir = new File(directory);

				// lists all folders in temperature folder (Refl / Trans).
				subDirs = dir.listFiles(new FileFilter() {
					public boolean accept(File pathname) {
						return pathname.isDirectory() && (!pathname.getName().equals(".svn"));
					}
				});
				options = new String[subDirs.length];

				int j = 0;
				for (int i = subDirs.length - 1; i >= 0; i--) {
					options[j] = subDirs[i].getName();
					j++;
				}
			}
			else
			{
				return new String[] {""};
			}
		} catch (Exception e) {
			logger.error(e.getMessage());
		}

		return options;
	}


	public static String[] readOpticalSettings() {


		try {

			String path = p3AppManagerUtils.formatString(p3Constants.getPath(p3Constants.OPTICAL_SETTINGS_FILE_PATH), UserInterface.moduleID);
			ArrayList<String> list = new ArrayList<String>();

			File filePath = new File(path);

			if(filePath.exists())
			{
				BufferedReader reader = null;
				try {
					reader = new BufferedReader(new FileReader(path));

					String line = null;
					while ((line = reader.readLine()) != null) {
						if(line.contains(":"))
						{
							line = line.split(":")[0];
							list.add(line);
						}
					}

				} catch (Exception ex) {
					return null;
				} finally {
					try {
						reader.close();
					} catch (Exception ex) {
						return null;
					}
				}
			}

			return (list.toArray(new String[list.size()]));


		} catch (Exception e) {
			logger.error(e.getMessage());
		}

		return null;
	}




	public static void deleteOpticalSetting(String setting)
	{
		try {

			String path = p3AppManagerUtils.formatString(p3Constants.getPath(p3Constants.OPTICAL_SETTINGS_FILE_PATH), UserInterface.moduleID);
			ArrayList<String> list = new ArrayList<String>();

			File filePath = new File(path);

			if(filePath.exists())
			{
				BufferedReader reader = null;
				try {
					reader = new BufferedReader(new FileReader(path));

					String line = null;
					while ((line = reader.readLine()) != null) {
						if(!line.split(":")[0].trim().equals(setting))
						{
							list.add(line);
						}
					}

				} catch (Exception ex) {
					return;
				} finally {
					try {
						reader.close();
					} catch (Exception ex) {
						return;
					}
				}

				BufferedWriter writer= null;
				try {
					writer = new BufferedWriter(new FileWriter(path, false));
					for(String line : list)
					{
						writer.write(line + "\r\n");
					}
				} catch (Exception ex) {
					return;
				} finally {
					try {
						writer.close();
					} catch (Exception ex) {
						return;
					}
				}
			}
			if(InterSpecPanel.lastOpticalSettingsSelected.equals(setting) 
					|| SpectroscopyPanel.lastOpticalSettingsSelected.equals(setting))
			{
				InterSpecPanel.lastOpticalSettingsSelected = "";
				SpectroscopyPanel.lastOpticalSettingsSelected = "";
			}
			
		} catch (Exception e) {
			logger.error(e.getMessage());
		}
	}

	public static String[] readStandardCalibrators() {
		String[] calibrators = null;
		String[] returnedCalibrators = null;
		try {
			String directory = p3Constants.getPath(p3Constants.STANDARD_CALIBRATORS_FOLDER_PATH);
			File dir = new File(directory);

			// lists all folders in directory.
			File[] subDirs = dir.listFiles();
			calibrators = new String[subDirs.length];
			int noCalibrators = 0;

			for (int i = 0; i< subDirs.length; i++) {
				if(subDirs[i].isFile() && subDirs[i].getName().contains(p3Constants.STANDARD_CALIBRATORS_FILE_EXT))
				{
					calibrators[noCalibrators] = subDirs[i].getName().replace(p3Constants.STANDARD_CALIBRATORS_FILE_EXT, "");
					noCalibrators++;
				}
			}

			returnedCalibrators = new String[noCalibrators];
			System.arraycopy(calibrators, 0, returnedCalibrators, 0, noCalibrators);

		} catch (Exception e) {
			logger.error(e.getMessage());
		}
		return returnedCalibrators;
	}

	

	public static void boardReadyRoutine(boolean ready) {
		if (ready) {
			SpectroscopyPanel.cmb_Optical_Settings_Spec.setEnabled(true);
			SpectroscopyPanel.btn_Run_Background.setEnabled(true);
			SpectroscopyPanel.btn_Run_Spec.setEnabled(true);
			SpectroscopyPanel.btn_Stop_Spec.setEnabled(true);
			SpectroscopyPanel.rdbtn_SingleRun_Spec.setEnabled(true);
			SpectroscopyPanel.rdbtn_ContRun_Spec.setEnabled(true);
			SpectroscopyPanel.btn_DeleteOpticalOption_Spec.setEnabled(true);
			SpectroscopyPanel.btn_SaveGraphs_Spec.setEnabled(true);
			SpectroscopyPanel.btn_LoadGraphs_Spec.setEnabled(true);
			//SpectroscopyPanel.btn_ClearGraphs_Spec.setEnabled(true);


			InterSpecPanel.cmb_Optical_Settings_Inter_Spec.setEnabled(true);
			InterSpecPanel.btn_Run_Inter_Spec.setEnabled(true);
			InterSpecPanel.btn_Stop_Inter_Spec.setEnabled(true);
			InterSpecPanel.rdbtn_SingleRun_Inter_Spec.setEnabled(true);
			InterSpecPanel.rdbtn_ContRun_Inter_Spec.setEnabled(true);
			InterSpecPanel.btn_DeleteOpticalOption_Inter_Spec.setEnabled(true);
			InterSpecPanel.btn_SaveGraphs_Inter_Spec.setEnabled(true);
			InterSpecPanel.btn_LoadGraphs_Inter_Spec.setEnabled(true);
			//InterSpecPanel.btn_ClearGraphs_Inter_Spec.setEnabled(true);

			InterSpecPanel.cmb_Apodization_Inter_Spec.setEnabled(true);
			InterSpecPanel.cmb_ZeroPadding_Inter_Spec.setEnabled(true);
			InterSpecPanel.btn_GainAdjustmentPanel_Inter_Spec.setEnabled(true);
			InterSpecPanel.btn_WavelengthWavenumberCalibration_Inter_Spec.setEnabled(true);			
			InterSpecPanel.btn_WriteSettings_Inter_Spec.setEnabled(true);
			InterSpecPanel.btn_RestoreDefault_Inter_Spec.setEnabled(true);

			SpectroscopyPanel.cmb_Apodization_Spec.setEnabled(true);
			SpectroscopyPanel.cmb_ZeroPadding_Spec.setEnabled(true);
			SpectroscopyPanel.btn_GainAdjustmentPanel_Spec.setEnabled(true);
			SpectroscopyPanel.btn_WavelengthWavenumberCalibration_Spec.setEnabled(true);
			SpectroscopyPanel.btn_WriteSettings_Spec.setEnabled(true);
			SpectroscopyPanel.btn_RestoreDefault_spec.setEnabled(true);
			
			UserInterface.chb_SleepAction_menuBar.setEnabled(true);
			UserInterface.chb_PowerOff_menuBar.setEnabled(true);

		} else {
			SpectroscopyPanel.cmb_Optical_Settings_Spec.setEnabled(false);
			SpectroscopyPanel.btn_Run_Background.setEnabled(false);
			SpectroscopyPanel.btn_Run_Spec.setEnabled(false);
			SpectroscopyPanel.rdbtn_SingleRun_Spec.setEnabled(false);
			SpectroscopyPanel.rdbtn_ContRun_Spec.setEnabled(false);
			SpectroscopyPanel.cmb_Apodization_Spec.setEnabled(false);
			SpectroscopyPanel.cmb_ZeroPadding_Spec.setEnabled(false);
			SpectroscopyPanel.btn_GainAdjustmentPanel_Spec.setEnabled(false);
			SpectroscopyPanel.btn_WavelengthWavenumberCalibration_Spec.setEnabled(false);			
			SpectroscopyPanel.btn_WriteSettings_Spec.setEnabled(false);
			SpectroscopyPanel.btn_RestoreDefault_spec.setEnabled(false);
			SpectroscopyPanel.btn_DeleteOpticalOption_Spec.setEnabled(false);
			SpectroscopyPanel.btn_SaveGraphs_Spec.setEnabled(false);
			SpectroscopyPanel.btn_LoadGraphs_Spec.setEnabled(false);
			//SpectroscopyPanel.btn_ClearGraphs_Spec.setEnabled(false);

			InterSpecPanel.cmb_Optical_Settings_Inter_Spec.setEnabled(false);
			InterSpecPanel.btn_Run_Inter_Spec.setEnabled(false);
			InterSpecPanel.rdbtn_SingleRun_Inter_Spec.setEnabled(false);
			InterSpecPanel.rdbtn_ContRun_Inter_Spec.setEnabled(false);
			InterSpecPanel.cmb_Apodization_Inter_Spec.setEnabled(false);
			InterSpecPanel.cmb_ZeroPadding_Inter_Spec.setEnabled(false);
			InterSpecPanel.btn_GainAdjustmentPanel_Inter_Spec.setEnabled(false);
			InterSpecPanel.btn_WavelengthWavenumberCalibration_Inter_Spec.setEnabled(false);
			InterSpecPanel.btn_WriteSettings_Inter_Spec.setEnabled(false);
			InterSpecPanel.btn_RestoreDefault_Inter_Spec.setEnabled(false);
			InterSpecPanel.btn_DeleteOpticalOption_Inter_Spec.setEnabled(false);
			InterSpecPanel.btn_SaveGraphs_Inter_Spec.setEnabled(false);
			InterSpecPanel.btn_LoadGraphs_Inter_Spec.setEnabled(false);
			//InterSpecPanel.btn_ClearGraphs_Inter_Spec.setEnabled(false);
			
			UserInterface.chb_SleepAction_menuBar.setEnabled(false);
			UserInterface.chb_PowerOff_menuBar.setEnabled(false);

		}
		ShowAdvancedSettings();
	}

	/*
	 * !reads optical settings from folder and displays them in InterSpec
	 */
	public static void displayInterSpecOpticalSettings() {
		logger.info("Reading InterSpec optical settings");

		InterSpecPanel.cmb_Optical_Settings_Inter_Spec.removeAllItems();
		
		InterSpecPanel.cmb_Optical_Settings_Inter_Spec.addItem("Default");
		
		String[] opticalSettings = readOpticalSettings();
		if (opticalSettings != null) {
			for (String s : opticalSettings) {
				InterSpecPanel.cmb_Optical_Settings_Inter_Spec.addItem(s);	
			}
		}
	}

	/*
	 * !reads optical settings from folder and displays them in Spec
	 */
	public static void displaySpecOpticalSettings() {
		logger.info("Reading Spec optical settings");

		SpectroscopyPanel.cmb_Optical_Settings_Spec.removeAllItems();

		SpectroscopyPanel.cmb_Optical_Settings_Spec.addItem("Default");
		
		String[] opticalSettings = readOpticalSettings();;
		if (opticalSettings != null) {
			for (String s : opticalSettings) {
				SpectroscopyPanel.cmb_Optical_Settings_Spec.addItem(s);

			}
		}
	}
	
	public static void displayCommonWVN() {
		String[] options = p3Constants.commonWavenumberOptions;
		
		SpectroscopyPanel.cmb_Wavenumber_No_Of_Points_Spec.removeAllItems();
		InterSpecPanel.cmb_Wavenumber_No_Of_Points_Inter_Spec.removeAllItems();
	
		for (String s : options) {
		
			SpectroscopyPanel.cmb_Wavenumber_No_Of_Points_Spec.addItem(s);
			InterSpecPanel.cmb_Wavenumber_No_Of_Points_Inter_Spec.addItem(s);
		
		}
		
	}


	/*
	 * !reads standard calibrators from standard_calibrator folder and display them in the calibration and spectroscopy tabs
	 */
	public static void displayStandardCalibrators() {
		String[] options = UserInterface.readStandardCalibrators();
		if (options != null) {
			try
			{
				WL_WN_CalibrationPanel.cmb_StandardCalibrator_WL_Calib.removeAllItems();
			}
			catch (Exception e) {

			}

			try
			{
				WL_WN_CalibrationPanel.cmb_StandardCalibrator_WL_Calib.addItem(p3Constants.STANDARD_CALIBRATOR_DEFAULT_CHOISE);
			}
			catch (Exception e) {

			}

			for (String s : options) {
				try
				{
					WL_WN_CalibrationPanel.cmb_StandardCalibrator_WL_Calib.addItem(s);
				}
				catch (Exception e) {

				}
			}
			try
			{
				WL_WN_CalibrationPanel.cmb_StandardCalibrator_WL_Calib.setSelectedIndex(0);
			}
			catch (Exception e) {

			}
		}
	}

	/*
	 * !reads standard calibrators from standard_calibrator folder and display them in the calibration and spectroscopy tabs
	 */
	public static boolean addNewStandardCalibrator(String newStandardCalibrator) {		
		newStandardCalibrator = newStandardCalibrator.replace(" ", "");
		newStandardCalibrator = newStandardCalibrator.replace("\t", "");

		if ((!newStandardCalibrator.contains("{")) || (!newStandardCalibrator.contains("}"))) {
			return false;
		}
		int calibratorNameLength = newStandardCalibrator.indexOf("{");
		int endOfString = newStandardCalibrator.indexOf("}");

		String calibratorName = newStandardCalibrator.substring(0, calibratorNameLength);

		if(!p3AppManagerUtils.isFilenameValid(calibratorName))
			return false;

		String calibratorValues = newStandardCalibrator.substring(calibratorNameLength + 1, endOfString);

		String[] wavenumbers = calibratorValues.split(",");

		for(String wavenumber : wavenumbers)
		{
			try{
				Double.parseDouble(wavenumber);
			}
			catch(Exception c){
				return false;
			}
		}

		try {
			BufferedWriter writer1 = new BufferedWriter(new FileWriter((p3Constants.getPath(p3Constants.STANDARD_CALIBRATORS_FOLDER_PATH) + File.separatorChar + calibratorName + p3Constants.STANDARD_CALIBRATORS_FILE_EXT)));

			for(int j = 0; j < wavenumbers.length; j++)
			{
				writer1.write(wavenumbers[j] + "\n");
			}
			writer1.close();
		} catch (IOException e) {
			e.printStackTrace();
		}

		return true;
	}

	public static void restoreSequence() {

		UserInterface.checkDeviceStatusThreadStop = true;
		// Start RUN
		while(acquireDeviceToken() == false);
		p3AppManagerStatus status = UserInterface.applicationManager.restoreDefaultSettings("All");
		if (p3AppManagerStatus.NO_ERROR != status) {
			JOptionPane.showMessageDialog(null,
					"Restore to default failed to start: : " + status,
					"Restore Default Settings", JOptionPane.OK_OPTION);

			if (p3AppManagerStatus.DEVICE_BUSY_ERROR != status) {
				// resume checking the status of the device
				UserInterface.checkDeviceStatusThreadStop = false;
				UserInterface.colorLabel.setForeground(Color.red);
				UserInterface.colorLabel.setBackground(Color.red);
			}

			// resume checking the status of the device
			UserInterface.releaseDeviceToken();
		} else {
			UserInterface.statusLabel.setText("Restoring to default started. Please wait...");

			UserInterface.colorLabel.setForeground(Color.yellow);
			UserInterface.colorLabel.setBackground(Color.yellow);
			UserInterface.boardReadyRoutine(false);
		}
		UserInterface.frmMain.revalidate();
		UserInterface.frmMain.repaint();

	}
	public static void burnSequence() {
		
		ButtonGroup buttinG = new ButtonGroup();
		JRadioButton opticalGain = new JRadioButton("Burn Gain Settings");
		JRadioButton correctionSelf= new JRadioButton("Burn Self Correction Settings");
		JRadioButton correctionWLN= new JRadioButton("Burn Reference Material Correction Settings");
		JRadioButton all = new JRadioButton("Burn All Settings");
		opticalGain.setSelected(true);
		
		buttinG.add(opticalGain);
		buttinG.add(correctionSelf);
		buttinG.add(correctionWLN);
		buttinG.add(all);
		
		JPanel panel = new JPanel();
		panel.setLayout(new MigLayout("", "[][][]",""));
		panel.add(opticalGain, "cell 0 0 1 1");
		panel.add(correctionSelf, "cell 0 1 1 1");
		panel.add(correctionWLN, "cell 0 2 1 1");
		panel.add(all, "cell 0 3 1 1");
		
		
		int n = JOptionPane.showConfirmDialog(
				null, panel, "Burn Settings", JOptionPane.OK_CANCEL_OPTION);

		String[] BurnMode = new String[1];
		if(opticalGain.isSelected())
			BurnMode[0] = "0";
		else if(correctionSelf.isSelected())
			BurnMode[0] = "1";
		else if(correctionWLN.isSelected())
			BurnMode[0] = "2";
		else if(all.isSelected())
			BurnMode[0] = "3";
		
		if (n == JOptionPane.OK_OPTION)
		{

			UserInterface.checkDeviceStatusThreadStop = true;
			// Start RUN
			while(acquireDeviceToken() == false);
			p3AppManagerStatus status = UserInterface.applicationManager.burnSpecificSettings(BurnMode);
			if (p3AppManagerStatus.NO_ERROR != status) {
				JOptionPane.showMessageDialog(null,
						"Burn Settings failed to start: : " + status,
						"Burn Settings", JOptionPane.OK_OPTION);

				if (p3AppManagerStatus.DEVICE_BUSY_ERROR != status) {
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

				}

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();
			} else {
				UserInterface.statusLabel.setText("Burn Settings started. Please wait...");

				UserInterface.colorLabel.setForeground(Color.yellow);
				UserInterface.colorLabel.setBackground(Color.yellow);
				UserInterface.boardReadyRoutine(false);
			}
			UserInterface.frmMain.revalidate();
			UserInterface.frmMain.repaint();
		}
	}
	
	public static void GainAdjustment()
	{

		logger.info("GainAdjustment function started");
		// Start RUN
		try{
			// stop checking the status of the device
			UserInterface.checkDeviceStatusThreadStop = true;

			UserInterface.progressTime = p3Constants.adaptiveGainRunTime;
			UserInterface.progressPar.setValue(0);
			logger.info("runSpecGainAdjBG function started");
			while(acquireDeviceToken() == false);
			p3AppManagerStatus status = UserInterface.applicationManager.runSpecGainAdjBG(Double.toString(p3Constants.adaptiveGainRunTime));
			logger.info("runSpecGainAdjBG function finished");
			if (p3AppManagerStatus.NO_ERROR != status) {

				if (p3AppManagerStatus.DEVICE_BUSY_ERROR != status) {
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;

					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);
				}

				JOptionPane.showMessageDialog(null,
						"Run failed to start: " + UserInterface.convertErrorCodesToMessages(status),
						"Gain Adjustment", JOptionPane.OK_OPTION);

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();

			} else {
				logger.info("disabling panel GUI fields started");
				UserInterface.boardReadyRoutine(false);
				logger.info("disabling panel GUI fields finished");
				UserInterface.statusLabel.setText(" Gain Adjustment function started. Please wait...");

				UserInterface.colorLabel.setForeground(Color.yellow);
				UserInterface.colorLabel.setBackground(Color.yellow);

			}
		}
		catch(Exception ex)
		{
			UserInterface.colorLabel.setForeground(Color.red);
			UserInterface.colorLabel.setBackground(Color.red);

			JOptionPane.showMessageDialog(null,
					"Run failed to start. Please check inputs.",
					"Gain Adjustment", JOptionPane.OK_OPTION);
		}
		UserInterface.frmMain.revalidate();
		UserInterface.frmMain.repaint();
	}
	
	/*
	 * !convert error codes into messages
	 */
	public static String convertErrorCodesToMessages(int errorCode) {
		switch(errorCode)
		{
		case 1:
			return "Error Code : " + errorCode + "\n NeoSpectra Micro kit is busy. \n Please wait until the current operation is done.";

		case 2:
			return "Error Code : " + errorCode + "\n SpectroMOST Micro does not detect any connected NeoSpectra Micro kit.";

		case 3:
			return "Error Code : " + errorCode + "\n NeoSpectra Micro kit is not initialized. \n Please disconnect and reconnect your kit.";

		case 4:
			return "Error Code : " + errorCode + "\n Unknown error. \n Please contact Si-Ware Systems.";

		case 7:
			return "Error Code : " + errorCode + "\n Error in loading configuration files. \n Please contact Si-Ware Systems.";

		case 8:
			return "Error Code : " + errorCode + "\n Error in configuration files' format. \n Please contact Si-Ware Systems.";

		case 11:
			return "Error Code : " + errorCode + "\n Invalid scan time. \n Please type a valid entry between 10 milliseconds and 28 seconds.";

		case 23:
			return "Error Code : " + errorCode + "\n Error in configuration files' format. \n Please contact Si-Ware Systems.";

		case 24:
			return "Error Code : " + errorCode + "\n Internal error in DSP. \n Please contact Si-Ware Systems.";

		case 25:
			return "Error Code : " + errorCode + "\n Internal error in DSP. \n Please contact Si-Ware Systems.";

		case 26:
			return "Error Code : " + errorCode + "\n Internal error in DSP. \n Please contact Si-Ware Systems.";

		case 27:
			return "Error Code : " + errorCode + "\n Internal error in DSP. \n Please contact Si-Ware Systems.";

		case 28:
			return "Error Code : " + errorCode + "\n Error in updating configuration files.\n Please contact Si-Ware Systems.";    

		case 29:
			return "Error Code : " + errorCode + "\n Error in saving data of background measurement. \n Please retry.";

		case 30:
			return "Error Code : " + errorCode + "\n Internal error in DSP. \n Please contact Si-Ware Systems.";

		case 31:
			return "Error Code : " + errorCode + "\n Invalid run parameters. \n Please use valid entries.";

		case 32:
			return "Error Code : " + errorCode + "\n The scan time of the background is different than the scan time of sample measurement. \n Please take a new background measurement.";

		case 33:
			return "Error Code : " + errorCode + "\n No valid background measurement found. \n Please take a new background measurement.";

		case 34:
			return "Error Code : " + errorCode + "\n Error occurred during saving data. \n Please try saving in a different directory.";

		case 35:
			return "Error Code : " + errorCode + "\n Error occurred during saving PSD data.\n Please try saving in a different directory.";

		case 36:
			return "Error Code : " + errorCode + "\n Error occurred during saving spectrum data.\n Please try saving in a different directory.";

		case 37:
			return "Error Code : " + errorCode + "\n Error occurred while creating folder to save measurement data. \n Please try creating the folder in a different directory.";

		case 38:
			return "Error Code : " + errorCode + "\n Invalid Apodization Window. \n Please use a valid apodization window.";

		case 42:
			return "Error Code : " + errorCode + "\n Error occurred during initialization of NeoSpectra Micro kit. \n Please disconnect and reconnect your kit.";

		case 43:
			return "Error Code : " + errorCode + "\n Error occurred during initialization of NeoSpectra Micro kit. \n Please disconnect and reconnect your kit.";

		case 50:
			return "Error Code : " + errorCode + "\n Error occurred during streaming from NeoSpectra Micro kit. \n Please disconnect and reconnect your kit.";

		case 51:
			return "Error Code : " + errorCode + "\n Error occurred during streaming from NeoSpectra Micro kit. \n Please disconnect and reconnect your kit.";

		case 52:
			return "Error Code : " + errorCode + "\n Error occurred during result return. \n Please make a new measurement.";

		case 53:
			return "Error Code : " + errorCode + "\n Invalid action !!";

		case 54:
			return "Error Code : " + errorCode + "\n Invalid device is connected.";

		case 55:
			return "Error Code : " + errorCode + "\n Internal threading error occurred. \n Please make a new measurement. \n If the error persists, please contact Si-Ware Systems.";

		case 60:
			return "Error Code : " + errorCode + "\n Error occurred during the setup of actuation settings. \n Please make a new measurement. \n If the error persists, please contact Si-Ware Systems.";

		case 61:
			return "Error Code : " + errorCode + "\n Actuation setting is turned off. \n Please make a new measurement. \n If the error persists, please contact Si-Ware Systems.";

		case 62:
			return "Error Code : " + errorCode + "\n Internal error occurred. \n Please contact Si-Ware Systems." ;

		case 110:
			return "Error Code : " + errorCode + "\n Internal error occurred while running adaptive gain. \n Please contact Si-Ware Systems." ;

		case 111:
			return "Error Code : " + errorCode + "\n Internal error occurred during reading registers from ASIC chip. \n Please contact Si-Ware Systems." ;

		case 112:
			return "Error Code : " + errorCode + " Failed to start. \nCorrection profile doesn't exist. \nPlease contact Si-Ware Systems.";

		case 113:
			return "Error Code : " + errorCode + "Failed to write optical gain settings. \n Please try again. \n If the error persists, please contact Si-Ware Systems.";

		case 114:
			return "Error Code : " + errorCode + "Failed to create optical gain settings file. \n Please try again. \n If the error persists, please contact Si-Ware Systems.";

		case 115:
			return "Error Code : " + errorCode + "Failed to load standard calibrator file. \n Please try again. \n If the error persists, please contact Si-Ware Systems.";

		case 116:
			return "Error Code : " + errorCode + "\n Internal error occurred while running wavelength calibration. \n Please contact Si-Ware Systems.";

		case 117:
			return "Error Code : " + errorCode + "SpectroMOST Micro didn't find any data to update. \n Please take a measurement first."; 

		case 118:
			return "Error Code : " + errorCode + "Internal error while updating results after changing FFT settings. \n Please contact Si-Ware Systems.";

		case 119:
			return "Error Code : " + errorCode + "\nSpectroMOST Micro didn't find the selected Optical gain option. \nPlease contact Si-Ware Systems.";

		default:
			return "Error Code : " + errorCode + "\nError occurred during result return.";
		}
	}

	/*
	 * !convert error codes into messages
	 */
	public static String convertErrorCodesToMessages(p3AppManagerStatus errorStatus) {
		return convertErrorCodesToMessages(errorStatus.getNumVal());
	}
	
	public static void ShowAdvancedSettings()
	{
		if(chb_Advanced_menuBar.isSelected()){
			
			SpectroscopyPanel.lbl_Apodization_Spec.setVisible(true);
			SpectroscopyPanel.lbl_ZeroPadding_Spec.setVisible(true);
			SpectroscopyPanel.separatorV1_Spec.setVisible(true);
			SpectroscopyPanel.lbl_FFT_Settings_Spec.setVisible(true);
			SpectroscopyPanel.lbl_advancedSettings_Spec.setVisible(true);
			SpectroscopyPanel.cmb_Apodization_Spec.setVisible(true);
			SpectroscopyPanel.cmb_ZeroPadding_Spec.setVisible(true);
			SpectroscopyPanel.btn_GainAdjustmentPanel_Spec.setVisible(true);
			SpectroscopyPanel.btn_WavelengthWavenumberCalibration_Spec.setVisible(true);
			SpectroscopyPanel.btn_WriteSettings_Spec.setVisible(true);
			SpectroscopyPanel.btn_RestoreDefault_spec.setVisible(true);
			
			InterSpecPanel.lbl_Apodization_Inter_Spec.setVisible(true);
			InterSpecPanel.lbl_ZeroPadding_Inter_Spec.setVisible(true);
			InterSpecPanel.separatorV1_Inter_Spec.setVisible(true);
			InterSpecPanel.lbl_FFT_Settings_Inter_Spec.setVisible(true);
			InterSpecPanel.lbl_advancedSettings_Inter_Spec.setVisible(true);
			InterSpecPanel.cmb_Apodization_Inter_Spec.setVisible(true);
			InterSpecPanel.cmb_ZeroPadding_Inter_Spec.setVisible(true);
			InterSpecPanel.btn_GainAdjustmentPanel_Inter_Spec.setVisible(true);
			InterSpecPanel.btn_WavelengthWavenumberCalibration_Inter_Spec.setVisible(true);			
			InterSpecPanel.btn_WriteSettings_Inter_Spec.setVisible(true);
			InterSpecPanel.btn_RestoreDefault_Inter_Spec.setVisible(true);
		}else{
			SpectroscopyPanel.lbl_Apodization_Spec.setVisible(false);
			SpectroscopyPanel.lbl_ZeroPadding_Spec.setVisible(false);
			SpectroscopyPanel.separatorV1_Spec.setVisible(false);
			SpectroscopyPanel.lbl_FFT_Settings_Spec.setVisible(false);
			SpectroscopyPanel.lbl_advancedSettings_Spec.setVisible(false);
			SpectroscopyPanel.cmb_Apodization_Spec.setVisible(false);
			SpectroscopyPanel.cmb_ZeroPadding_Spec.setVisible(false);
			SpectroscopyPanel.btn_GainAdjustmentPanel_Spec.setVisible(false);
			SpectroscopyPanel.btn_WavelengthWavenumberCalibration_Spec.setVisible(false);
			SpectroscopyPanel.btn_WriteSettings_Spec.setVisible(false);
			SpectroscopyPanel.btn_RestoreDefault_spec.setVisible(false);
			
			InterSpecPanel.lbl_Apodization_Inter_Spec.setVisible(false);
			InterSpecPanel.lbl_ZeroPadding_Inter_Spec.setVisible(false);
			InterSpecPanel.separatorV1_Inter_Spec.setVisible(false);
			InterSpecPanel.lbl_FFT_Settings_Inter_Spec.setVisible(false);
			InterSpecPanel.lbl_advancedSettings_Inter_Spec.setVisible(false);
			InterSpecPanel.cmb_Apodization_Inter_Spec.setVisible(false);
			InterSpecPanel.cmb_ZeroPadding_Inter_Spec.setVisible(false);
			InterSpecPanel.btn_GainAdjustmentPanel_Inter_Spec.setVisible(false);
			InterSpecPanel.btn_WavelengthWavenumberCalibration_Inter_Spec.setVisible(false);			
			InterSpecPanel.btn_WriteSettings_Inter_Spec.setVisible(false);
			InterSpecPanel.btn_RestoreDefault_Inter_Spec.setVisible(false);
		}
	}
	
	public static void runSleepAction()
	{
		if(chb_SleepAction_menuBar.isSelected()){
			UserInterface.checkDeviceStatusThreadStop = true;
			// Start RUN
			while(acquireDeviceToken() == false);
			p3AppManagerStatus status = UserInterface.applicationManager.sleepAction();
			if (p3AppManagerStatus.NO_ERROR != status) {
				JOptionPane.showMessageDialog(null,
						"Sleep Action failed to start: : " + status,
						"Sleep Action", JOptionPane.OK_OPTION);

				if (p3AppManagerStatus.DEVICE_BUSY_ERROR != status) {
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

				}

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();
			} else {
				UserInterface.statusLabel.setText("Sleep Action started. Please wait...");

				UserInterface.colorLabel.setForeground(Color.yellow);
				UserInterface.colorLabel.setBackground(Color.yellow);
				UserInterface.boardReadyRoutine(false);
			}
			UserInterface.frmMain.revalidate();
			UserInterface.frmMain.repaint();
		}else{
			UserInterface.checkDeviceStatusThreadStop = true;
			// Start RUN
			while(acquireDeviceToken() == false);
			p3AppManagerStatus status = UserInterface.applicationManager.wakeUpAction();
			if (p3AppManagerStatus.NO_ERROR != status) {
				JOptionPane.showMessageDialog(null,
						"Wake up Action failed to start: : " + status,
						"Wake up Action", JOptionPane.OK_OPTION);

				if (p3AppManagerStatus.DEVICE_BUSY_ERROR != status) {
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

				}

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();
			} else {
				UserInterface.statusLabel.setText("Wake up Action started. Please wait...");

				UserInterface.colorLabel.setForeground(Color.yellow);
				UserInterface.colorLabel.setBackground(Color.yellow);
				UserInterface.boardReadyRoutine(false);
			}
			UserInterface.frmMain.revalidate();
			UserInterface.frmMain.repaint();
		}
	}
	
	public static void runPowerOff()
	{
		if(chb_PowerOff_menuBar.isSelected()){
			UserInterface.checkDeviceStatusThreadStop = true;
			// Start RUN
			while(acquireDeviceToken() == false);
			p3AppManagerStatus status = UserInterface.applicationManager.powerOff();
			if (p3AppManagerStatus.NO_ERROR != status) {
				JOptionPane.showMessageDialog(null,
						"Power off failed to start: : " + status,
						"Power off", JOptionPane.OK_OPTION);

				if (p3AppManagerStatus.DEVICE_BUSY_ERROR != status) {
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

				}

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();
			} else {
				UserInterface.statusLabel.setText("Power Off started. Please wait...");

				UserInterface.colorLabel.setForeground(Color.yellow);
				UserInterface.colorLabel.setBackground(Color.yellow);
				UserInterface.boardReadyRoutine(false);
			}
			UserInterface.frmMain.revalidate();
			UserInterface.frmMain.repaint();
		}else{
			UserInterface.checkDeviceStatusThreadStop = true;
			// Start RUN
			while(acquireDeviceToken() == false);
			p3AppManagerStatus status = UserInterface.applicationManager.powerOn();
			if (p3AppManagerStatus.NO_ERROR != status) {
				JOptionPane.showMessageDialog(null,
						"Power on failed to start: : " + status,
						"Power on", JOptionPane.OK_OPTION);

				if (p3AppManagerStatus.DEVICE_BUSY_ERROR != status) {
					// resume checking the status of the device
					UserInterface.checkDeviceStatusThreadStop = false;
					UserInterface.colorLabel.setForeground(Color.red);
					UserInterface.colorLabel.setBackground(Color.red);

				}

				// resume checking the status of the device
				UserInterface.releaseDeviceToken();
			} else {
				UserInterface.statusLabel.setText("Power on started. Please wait...");

				UserInterface.colorLabel.setForeground(Color.yellow);
				UserInterface.colorLabel.setBackground(Color.yellow);
				UserInterface.boardReadyRoutine(false);
			}
			UserInterface.frmMain.revalidate();
			UserInterface.frmMain.repaint();
		}
	}

}
