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
import sws.p3AppManager_micro.utils.p3Constants;

import javax.swing.JTextField;

import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class SaveOpticalGainPanel extends JPanel {

	static JFrame frmSaveOpticalGain;
	static JButton btn_Save;

	private JLabel lblSetupName;
	private JTextField txt_SetupName;
	private JLabel lblNote;
	
	private int gainValue;
	
	public SaveOpticalGainPanel(int gainValue) {
		super();
		this.gainValue = gainValue;
		this.initialize();
	}

	public void initialize() {

		frmSaveOpticalGain = new JFrame();

		try {
			File imgFile = new File(UserInterface.ICON_PATH);
			if (imgFile.exists()) {
				Image img = ImageIO.read(imgFile);
				frmSaveOpticalGain.setIconImage(img);
			} else {
				JOptionPane.showMessageDialog(frmSaveOpticalGain, "Icon doesn't exist.",
						"Loading image", JOptionPane.OK_OPTION);
			}
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(frmSaveOpticalGain,
					"Error loading Icon:" + ex.getMessage(), "Loading icon",
					JOptionPane.OK_OPTION);
		}

		frmSaveOpticalGain.setBackground(new Color(176, 196, 222));
		frmSaveOpticalGain.getContentPane().setBackground(new Color(176, 196, 222));
		frmSaveOpticalGain.setVisible(true);
		frmSaveOpticalGain.setName("SpecFrame");

		frmSaveOpticalGain.setMinimumSize(new Dimension((int)(p3Constants.SUB_PANELS_DIMENTION * 1.5), (int)(p3Constants.SUB_PANELS_DIMENTION * 0.4)));
		frmSaveOpticalGain.setMaximumSize(new Dimension((int)(p3Constants.SUB_PANELS_DIMENTION * 1.5), (int)(p3Constants.SUB_PANELS_DIMENTION * 0.4)));
		frmSaveOpticalGain.setPreferredSize(new Dimension((int)(p3Constants.SUB_PANELS_DIMENTION * 1.5), (int)(p3Constants.SUB_PANELS_DIMENTION * 0.4)));
		frmSaveOpticalGain.setLocation((int)(UserInterface.screenSize.getWidth()/2 - p3Constants.SUB_PANELS_DIMENTION / 2), (int)(UserInterface.screenSize.getHeight()/2 - p3Constants.SUB_PANELS_DIMENTION * 0.2));
		
		frmSaveOpticalGain.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent arg0) {
				// Exit
				onCloseOperation();
			}
		});
		frmSaveOpticalGain.setTitle("Saving Optical Gain Value");
		frmSaveOpticalGain.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		frmSaveOpticalGain.setResizable(false);

		frmSaveOpticalGain.setFont(new Font("Dialog", Font.PLAIN, 12));
		frmSaveOpticalGain.setBackground(new Color(176, 196, 222));
		frmSaveOpticalGain.getContentPane().setLayout(new MigLayout("", "[][][][][45.00:45.00,grow]", "[][][]"));

		lblSetupName = new JLabel("Optical Gain Name");
		frmSaveOpticalGain.getContentPane().add(lblSetupName, "cell 0 0 2 1,grow");

		txt_SetupName = new JTextField();
		txt_SetupName.addKeyListener(new KeyAdapter() {
			@Override
			public void keyPressed(KeyEvent arg0) {
				//if Enter is pressed
				if((int)arg0.getKeyChar() == 10)
				{
					saveAction();
				}
			}
		});
		frmSaveOpticalGain.getContentPane().add(txt_SetupName, "cell 2 0 3 1,growx");
		txt_SetupName.setColumns(10);

		lblNote = new JLabel("Please enter the name of your setup");
		frmSaveOpticalGain.getContentPane().add(lblNote, "cell 0 1 5 1,grow");

		btn_Save = new JButton("");
		btn_Save.setFont(new Font("Dialog", Font.PLAIN, 12));
		btn_Save.setText("Save");
		btn_Save.setVerticalTextPosition(SwingConstants.CENTER);
		btn_Save.setHorizontalTextPosition(SwingConstants.CENTER);
		btn_Save.setToolTipText("Save Optical Gain Value");
		btn_Save.setMaximumSize(new Dimension((int)(p3Constants.MAX_WIDTH_OF_FIELD), btn_Save.getPreferredSize().height));

		btn_Save.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				saveAction();
			}
		});
		frmSaveOpticalGain.getContentPane().add(btn_Save, "cell 0 2 5 1,alignx center");
	}

	private void onCloseOperation()
	{
		// resume checking the status of the device
		UserInterface.releaseDeviceToken();
		UserInterface.checkDeviceStatusThreadStop = false;
		UserInterface.boardReadyRoutine(true);
		
		frmSaveOpticalGain.dispose();
	}
	
	private void saveAction()
	{
		int status;
		String name = "";
		if(!txt_SetupName.getText().equals(""))
		{
			if(txt_SetupName.getText().contains(":")){
				name = txt_SetupName.getText().replace(":", "");
			}
			else{
				name = txt_SetupName.getText();
			}
			status = UserInterface.saveOpticalGainRoutine(name, this.gainValue);
			if(status == 1){
				int result = JOptionPane.showConfirmDialog(
						this,
						"There is already a setup with the same name. do you want to overwrite it?",
						"Name already exists",
						JOptionPane.YES_NO_OPTION);
				switch (result) {
				case JOptionPane.YES_OPTION:
					UserInterface.deleteOpticalSetting(name);
					UserInterface.saveOpticalGainRoutine(name, this.gainValue);
					break;
				case JOptionPane.CLOSED_OPTION:
					return;
				case JOptionPane.NO_OPTION:
					return;
				}
			}else if(status < 0){
				JOptionPane.showMessageDialog(null,
						"Optical gain value can't be saved, please try again.",
						"Saving error", JOptionPane.OK_OPTION);
				return;
			}
		}else
		{
			JOptionPane.showMessageDialog(null,
					"Please fill the name field!",
					"Saving Optical Gain Value", JOptionPane.OK_OPTION);
			
			return;
		}

		UserInterface.displaySpecOpticalSettings();
		UserInterface.displayInterSpecOpticalSettings();
		UserInterface.updateSelectedOpticalSetting();
		onCloseOperation();
	}

}