package sws.spectromost;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Image;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.SwingConstants;
import javax.swing.JLabel;

import net.miginfocom.swing.MigLayout;
import sws.p3AppManager_micro.utils.p3AppManagerUtils;
import sws.p3AppManager_micro.utils.p3Constants;

import javax.swing.JTextField;

import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class SaveGraphsPanel extends JPanel {

    static JFrame frmSaveGraphs;
	static JButton btn_Save;

	private JLabel lblFileName;
	private JTextField txt_FileName;
	private JLabel lblNote;
	
	private static String tabPrefix = null;
	
	public SaveGraphsPanel(String tabPrefix) {
		super();
		SaveGraphsPanel.tabPrefix = tabPrefix;
		UserInterface.fileNameToSave = "!Cancel!";
		this.initialize();
	}

	public void initialize() {

		frmSaveGraphs = new JFrame();
		frmSaveGraphs.setAlwaysOnTop (true);

		try {
			File imgFile = new File(UserInterface.ICON_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				frmSaveGraphs.setIconImage(img);
			} else {
				JOptionPane.showMessageDialog(frmSaveGraphs, "Icon doesn't exist.",
						"Loading image", JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(frmSaveGraphs,
					"Error loading Icon:" + ex.getMessage(), "Loading icon",
					JOptionPane.OK_OPTION);
		}

		frmSaveGraphs.setBackground(new Color(176, 196, 222));
		frmSaveGraphs.getContentPane().setBackground(new Color(176, 196, 222));
		frmSaveGraphs.setVisible(true);
		frmSaveGraphs.setName("SpecFrame");

		frmSaveGraphs.setMinimumSize(new Dimension((int)(p3Constants.SUB_PANELS_DIMENTION * 1.5), (int)(p3Constants.SUB_PANELS_DIMENTION * 0.4)));
		frmSaveGraphs.setMaximumSize(new Dimension((int)(p3Constants.SUB_PANELS_DIMENTION * 1.5), (int)(p3Constants.SUB_PANELS_DIMENTION * 0.4)));
		frmSaveGraphs.setPreferredSize(new Dimension((int)(p3Constants.SUB_PANELS_DIMENTION * 1.5), (int)(p3Constants.SUB_PANELS_DIMENTION * 0.4)));
		frmSaveGraphs.setLocation((int)(UserInterface.screenSize.getWidth()/2 - p3Constants.SUB_PANELS_DIMENTION / 2), (int)(UserInterface.screenSize.getHeight()/2 - p3Constants.SUB_PANELS_DIMENTION * 0.2));
		
		frmSaveGraphs.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent arg0) {
				// Exit
				onCloseOperation();
			}
		});
		frmSaveGraphs.setTitle("Saving Graphs");
		frmSaveGraphs.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		frmSaveGraphs.setResizable(false);

		frmSaveGraphs.setFont(new Font("Dialog", Font.PLAIN, 12));
		frmSaveGraphs.setBackground(new Color(176, 196, 222));
		frmSaveGraphs.getContentPane().setLayout(new MigLayout("", "[90.00:90.00][45.00:45.00,grow]", "[][][]"));

		lblFileName = new JLabel("File Name");
		frmSaveGraphs.getContentPane().add(lblFileName, "cell 0 0,grow");

		txt_FileName = new JTextField();
		txt_FileName.addKeyListener(new KeyAdapter() {
			@Override
			public void keyPressed(KeyEvent arg0) {
				//if Enter is pressed
				if((int)arg0.getKeyChar() == 10)
				{
					saveAction();
				}
			}
		});
		frmSaveGraphs.getContentPane().add(txt_FileName, "cell 1 0,growx");
		txt_FileName.setColumns(10);

		lblNote = new JLabel("Note: Measurements' original \r\nnames will be used as default.");
		frmSaveGraphs.getContentPane().add(lblNote, "cell 0 1 2 1,grow");

		btn_Save = new JButton("");
		btn_Save.setFont(new Font("Dialog", Font.PLAIN, 12));
		btn_Save.setText("Save");
		btn_Save.setVerticalTextPosition(SwingConstants.CENTER);
		btn_Save.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_Save.setToolTipText("Save Graphs");
		btn_Save.setMaximumSize(new Dimension((int)(p3Constants.MAX_WIDTH_OF_FIELD), btn_Save.getPreferredSize().height));

		btn_Save.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				saveAction();
			}
		});
		frmSaveGraphs.getContentPane().add(btn_Save, "cell 0 2 2 1,alignx center");
	}

	private void onCloseOperation()
	{
		UserInterface.stopEnablingButtons = false;
		// resume checking the status of the device
		UserInterface.checkDeviceStatusThreadStop = false;
		UserInterface.boardReadyRoutine(true);
		frmSaveGraphs.dispose();
		
		InterSpecPanel.btn_ClearGraphs_Inter_Spec.setEnabled(true);
		
		SpectroscopyPanel.btn_ClearGraphs_Spec.setEnabled(true);
		
		if(tabPrefix.equals(p3Constants.InterSpecPrefix) && !UserInterface.fileNameToSave.equals("!Cancel!"))
		{
			InterSpecPanel.saveGraphsRoutine();
		}
		else if(tabPrefix.equals(p3Constants.SpecPrefix) && !UserInterface.fileNameToSave.equals("!Cancel!"))
		{
			SpectroscopyPanel.saveGraphsRoutine();
		}
	}
	
	private void saveAction()
	{
		if(txt_FileName.getText().equals(""))
		{
			UserInterface.fileNameToSave = "!Default!";
		}
		else if(p3AppManagerUtils.isFilenameValid(txt_FileName.getText()))
		{
			UserInterface.fileNameToSave = txt_FileName.getText();
		}
		else
		{
			JOptionPane.showMessageDialog(null,
					"The name you entered isn't valid for file naming!",
					"Saving Graphs", JOptionPane.OK_OPTION);
			
			return;
		}

		onCloseOperation();
	}

}