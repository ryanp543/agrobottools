package sws.p3AppManager_micro;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import sws.p3AppManager_micro.configuration.p3RunConfig;
import sws.p3AppManager_micro.utils.p3AppManagerException;
import sws.p3AppManager_micro.utils.p3Enumerations.p3AppManagerState;

class RequestPacket
{
    public int  operation;
    public int  resolution;
    public int  mode;
    public int  zeroPadding;
    public int  scanTime;
    public int  commonWaveNum;
    public int  opticalGain;
    public int  apodizationSel;
    public int[]  calibrationWells = new int[40];
    
    public byte[] getBytes() throws IOException
    {
        ByteArrayOutputStream b = new ByteArrayOutputStream();
        b.write(ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(operation).array());
        b.write(ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(resolution).array());
        b.write(ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(mode).array());
        b.write(ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(zeroPadding).array());
        b.write(ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(scanTime).array());
        b.write(ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(commonWaveNum).array());
        b.write(ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(opticalGain).array());
        b.write(ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(apodizationSel).array());
        
        for(int i=0; i< calibrationWells.length; i++)
            b.write(ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(calibrationWells[i]).array());

        return b.toByteArray();
    }

}

class PSDResponse
{
    public int    status;
    public int    length;
    public long[] PSD = new long[4096];
    public long[] WVN_WVL = new long[4096];
     
    public PSDResponse(byte[] data)
    {
    	status = ByteBuffer.wrap(data, 0, 4).order(ByteOrder.LITTLE_ENDIAN).getInt();
    	length = ByteBuffer.wrap(data, 4, 4).order(ByteOrder.LITTLE_ENDIAN).getInt();
    	for(int i = 0; i < 4096; i++)
    	{
    		PSD[i] = ByteBuffer.wrap(data, 8 + i*8, 8).order(ByteOrder.LITTLE_ENDIAN).getLong();
    		WVN_WVL[i] = ByteBuffer.wrap(data, 32768 + 8 + i*8, 8).order(ByteOrder.LITTLE_ENDIAN).getLong();
    	}
	}

}

class P3Thread extends Thread{
	
	boolean running = true;
	byte[] data = null;
	Exception threadException = null;

	public P3Thread() {
		super();
	}
	
	public P3Thread(String string) {
		super(string);
	}

	public void terminate()
	{
		running = false;
		this.interrupt();
	}

	public synchronized byte[] getData()
	{
		return data;
	}
	
	public synchronized Exception getException()
	{
		return threadException;
	}
}

public class P3Driver {
	
	private Socket readSocket, writeSocket = null;

	
	private static final String serverIP = "192.168.137.2";
	private static final int readPort = 5001;
	private static final int writePort = 5000;
	
	private boolean initialized = false;
	
	
	
	public P3Driver(int productId)
	{

	}	

	public P3Driver(int productId, String dir)
	{

	}

	/*!
	 * This method is responsible for the board initialization. 
	 * It should start the board initialization. Before board initialization, 
	 * it should check first if the board is connected and if 
	 * the firmware version check is passed or else throw an exception identifying the error 
	 * 
	 * @throws TAIFException
	 */
	public void initiateTAIF(boolean... args) 
	{
		initialized = true;
	}

	
	private byte[] receivePacket() throws Exception
	{
		
		if(readSocket == null)
		{
			readSocket = new Socket(serverIP, readPort);
		}

		P3Thread connectionTestThread = new P3Thread() {
			@Override
			public void run() {
				
				int i = 0;
				int trial = 0;
				while(running)
				{
					try {
						Socket testSocket = new Socket();
						testSocket.connect(new InetSocketAddress(serverIP, writePort), 1000);

						Thread.sleep(1000);
						testSocket.close();
						i++;
					} 
					catch (IOException e) {
						System.out.println(e.getMessage() + i);
						if(trial == 1)
						{
							this.data = new byte[1];
							data[0] = (byte)0;
							break;
						}
						trial++;
					}
					catch (InterruptedException e) {
						break;
					}
				}
			}
		};
		connectionTestThread.start();
		
		P3Thread receiveThread = new P3Thread() {
			@Override
			public void run() {
				byte[] buffer = new byte[100000];
				byte[] buffersize = new byte[4];
				byte[] result;
				int offset = 0, ret = 0;

				try{
					readSocket.setSoTimeout(0);
					ret = readSocket.getInputStream().read(buffer, offset, buffer.length - offset);
					if(ret > 0)
					{
						offset += ret;
						
						for(int i = 0; i < buffersize.length; i++){
							buffersize[buffersize.length - 1 - i] = buffer[i];
						}
						int length = ByteBuffer.wrap(buffersize).getInt();
						
						while(offset < (length + 4))
				    	{
							ret = readSocket.getInputStream().read(buffer, offset, buffer.length - offset);
							offset += ret;
				    	}
						result = new byte[offset - 4];
						System.arraycopy(buffer, 4, result, 0, offset - 4);
						data = result;
					}else{
						data = new byte[0];
					}
				}
				catch(Exception e){
					threadException = e;
				}
			}
		};
		receiveThread.start();
		
		while(true)
		{
			if(receiveThread.getData() != null)
			{
				break;
			}
			
			if(connectionTestThread.getData() != null)
			{
				System.out.println("connectionTest Thread not null");
				throw new Exception("Error DVK is unplugged");
			}
				
			if(receiveThread.getException() != null)
			{
				System.out.println("receive Thread Exception");
				connectionTestThread.terminate();
				connectionTestThread.join();
				throw receiveThread.getException();
			}
			
		}

		connectionTestThread.terminate();
		connectionTestThread.join();
		return receiveThread.getData();
	}
	
	private byte[] receivePacket(int timeout) throws Exception
	{
		byte[] buffer = new byte[100000];
		byte[] buffersize = new byte[4];
		byte[] result;
		int offset = 0, ret = 0;
		
		if(readSocket == null)
		{
			readSocket = new Socket(serverIP, readPort);
		}
		
		readSocket.setSoTimeout(timeout);
		ret = readSocket.getInputStream().read(buffer, offset, buffer.length - offset);
		if(ret > 0)
		{
			offset += ret;
			
			for(int i = 0; i < buffersize.length; i++){
				buffersize[buffersize.length - 1 - i] = buffer[i];
			}
			int length = ByteBuffer.wrap(buffersize).getInt();
			
			while(offset < (length + 4))
	    	{
				ret = readSocket.getInputStream().read(buffer, offset, buffer.length - offset);
				offset += ret;
	    	}
			result = new byte[offset - 4];
			System.arraycopy(buffer, 4, result, 0, offset - 4);
			return result;
		}else{
			return new byte[0];
		}
	}
	
	private void sendPacket(RequestPacket rp) throws Exception
	{
		writeSocket.getOutputStream().write(rp.getBytes());
		Thread.sleep(10);
	}

	/*!
	 * Check board status, if it is connected and initialized
	 * return 0 if the board is connected and initialized 
	 * else returns the appropriate error code 
	 * (currently 14 if board is disconnected and 74 if it is not initialized)
	 */
	public int checkBoardStatus()
	{
		int boardStatus = 0;
	
	    try 
	    {
	    	if(writeSocket == null)
	    	{
	    		writeSocket = new Socket();//(serverIP, writePort);
	    		writeSocket.connect(new InetSocketAddress(serverIP, writePort), 200);
	    		
	    		try{
					
					byte[] buffer = checkBoard();
					if(buffer[0] == 1)
			    		boardStatus = 74;
					
				}
				catch(Exception e)
				{
		    		System.out.println("Catch 1");
					writeSocket = null;
					readSocket = null;
					initialized = false;
		    		boardStatus = 14;
				}
	    		
	    		
	    	}
	    	else if(initialized)
	    	{
	    		try{
				
					byte[] buffer = checkBoard();
					if(buffer[0] == 1)
						boardStatus = 0;
				}
				catch(Exception e)
				{
					boardStatus = 14;
					writeSocket = null;
					readSocket = null;
					initialized = false;
				}
	    	}
	    	else
	    	{
	    		boardStatus = 74;
	    	}
	    }
	    catch(Exception e)
	    {
	    	boardStatus = 14;
			writeSocket = null;
			readSocket = null;
			initialized = false;
	    }
	    
		return boardStatus;
	}
	
	public double[][] runOperation(p3RunConfig runconfig) throws Exception
	{
		runconfig.zeroPaddingMultiplier += 1; // Add zero padding offset
		runconfig.mode *= 5; // single mode = 0, continuous mode (pipelined) PSD and Wave number = 4, continuous mode (switching) PSD and Wave number = 5
		
		if(!runconfig.runType.equals(p3AppManagerState.BurnSettings))
			setOpticalSettings(runconfig.gainValue);
		SourceSettings(runconfig.t1, runconfig.t2_c1, runconfig.t2_c2, runconfig.t2_tmax, runconfig.delta_t, runconfig.lamps_count, runconfig.lamp_sel);
		
		if(runconfig.apodizationIndex == 4)
			injectExternalWindow(runconfig.externalApodizationWindow);
		
		switch(runconfig.runType)
		{
		case InterferogramRun:
			return runPSD((int)runconfig.runTime, 
					runconfig.apodizationIndex, runconfig.zeroPaddingMultiplier, runconfig.gainIndex, 
					runconfig.commonWaveNum, runconfig.mode);
		case SpectroscopyBackgroundRun:
			return runBackground((int)runconfig.runTime, 
					runconfig.apodizationIndex, runconfig.zeroPaddingMultiplier, runconfig.gainIndex, 
					runconfig.commonWaveNum);
		case SpectroscopySampleRun:
			return runAbsorbance((int)runconfig.runTime, 
					runconfig.apodizationIndex, runconfig.zeroPaddingMultiplier, runconfig.gainIndex, 
					runconfig.commonWaveNum, runconfig.mode);
		case gainAdjustSpecBG_Run:
			return runGainAdjusment();
		case BurnSettings:
			return runBurnSettings(runconfig.burnMode);
		case selfCorr_Run:
			return runSelfCorr((int)runconfig.runTime, 
					runconfig.apodizationIndex, runconfig.zeroPaddingMultiplier, runconfig.gainIndex, 
					runconfig.commonWaveNum);
		case wavelengthCalibrationBG_Run:
			return runWavelengthCorrBG((int)runconfig.runTime, 
					runconfig.apodizationIndex, runconfig.zeroPaddingMultiplier, runconfig.gainIndex, 
					runconfig.commonWaveNum);
		case wavelengthCalibration_Run:
			return runWavelengthCorr((int)runconfig.runTime, 
					runconfig.apodizationIndex, runconfig.zeroPaddingMultiplier, runconfig.gainIndex, 
					runconfig.commonWaveNum, runconfig.CalibrationWells);
		case RestoreDefault:
			return runRestoreDefault();
		case SleepAction:
			return sleepAction();
		case WakeUpAction:
			return wakeUpAction();
		case PowerOff:
			return powerOff();
		case PowerOn:
			return powerOn();

		default: 
			return null;
		}

	}

	private void injectExternalWindow(long []externalApodizationWindow) throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 28;
		
		{
			rp.scanTime = 0;
			rp.apodizationSel = 0;
			rp.zeroPadding = 0;
			rp.commonWaveNum = 0; 
			rp.resolution = 0;
		    rp.mode = 0;
		    rp.opticalGain = 0;
		}
		
		for(int i = 0; i < externalApodizationWindow.length; i++)
		{
			rp.calibrationWells[2 * i] = (int)(externalApodizationWindow[i] & 0xFFFFFFFF);
			rp.calibrationWells[2 * i + 1] = (int)((externalApodizationWindow[i] >>> 32) & 0xFFFFFFFF);
		}
		
		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
		
	}
	
	private double[][] runPSD(int runTime, int apodizationIndex, int zeroPadding, int gainIndex, int commonWaveNum, int mode) throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 3;
		
		{
			rp.scanTime = runTime;
			rp.apodizationSel = apodizationIndex;
			rp.zeroPadding = zeroPadding;
			rp.commonWaveNum = commonWaveNum; 
			rp.resolution = 0;
		    rp.mode = mode;
		    rp.opticalGain = gainIndex;
		}

		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		
		PSDResponse PSDres;
		try {
			PSDres = new PSDResponse(receivePacket());
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(PSDres.status != 0)
			throw new p3AppManagerException("DVK error. ", (int)(PSDres.status + 200));
		

		
		double[][] result = new double[5][];

		result[0] = new double[PSDres.length]; 
		result[1] = new double[PSDres.length];
		result[2] = new double[PSDres.length];
		result[3] = new double[PSDres.length];
		result[4] = new double[1];
		
		for(int i = 0; i < PSDres.length; i++)
		{
			result[0][i] = ((double)PSDres.WVN_WVL[i]) / Math.pow(2.0, 30);
			result[1][i] = ((double)PSDres.PSD[i]) /  Math.pow(2.0, 33);
			result[2][i] = ((double)PSDres.WVN_WVL[i]) /  Math.pow(2.0, 30);
			result[3][i] = ((double)PSDres.PSD[i]) /  Math.pow(2.0, 33);
		}
		
		result[4][0] = PSDres.length;
		
		return result;
	}
	
	private double[][] runBackground(int runTime, int apodizationIndex, int zeroPadding, int gainIndex, int commonWaveNum) throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 4;
		
		{
			rp.scanTime = runTime;
			rp.apodizationSel = apodizationIndex;
			rp.zeroPadding = zeroPadding;
			rp.commonWaveNum = commonWaveNum; 
			rp.resolution = 0;
		    rp.mode = 0;
		    rp.opticalGain = gainIndex;
		}
		
		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}

		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
		
		double[][] result = new double[1][];
		result[0] = new double[1];
		
		result[0][0] = (double)response[0];

		return result;
	}
	
	private double[][] runAbsorbance(int runTime, int apodizationIndex, int zeroPadding, int gainIndex, int commonWaveNum, int mode) throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 5;
		
		{
			rp.scanTime = runTime;
			rp.apodizationSel = apodizationIndex;
			rp.zeroPadding = zeroPadding;
			rp.commonWaveNum = commonWaveNum; 
			rp.resolution = 0;
		    rp.mode = mode;
		    rp.opticalGain = gainIndex;
		}

		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		PSDResponse PSDres;
		try {
			PSDres = new PSDResponse(receivePacket());
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(PSDres.status != 0)
			throw new p3AppManagerException("DVK error. ", (int)(PSDres.status + 200));
		
		double[][] result = new double[5][];

		result[0] = new double[PSDres.length]; 
		result[1] = new double[PSDres.length];
		result[2] = new double[PSDres.length];
		result[3] = new double[PSDres.length];
		result[4] = new double[1];
		
		for(int i = 0; i < PSDres.length; i++)
		{
			result[0][i] = ((double)PSDres.WVN_WVL[i]) / Math.pow(2.0, 30);
			result[1][i] = ((double)PSDres.PSD[i]) /  Math.pow(2.0, 33);
			result[2][i] = ((double)PSDres.WVN_WVL[i]) /  Math.pow(2.0, 30);
			result[3][i] = ((double)PSDres.PSD[i]) /  Math.pow(2.0, 33);
		}
		
		result[4][0] = PSDres.length;
		return result;
	}

	private double[][] runGainAdjusment() throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 6;
		byte[] buffersize = new byte[2];
		
		{
			rp.scanTime = 0;
			rp.apodizationSel = 0;
			rp.zeroPadding = 0;
			rp.commonWaveNum = 0; 
			rp.resolution = 0;
		    rp.mode = 0;
		    rp.opticalGain = 0;
		}

		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
		
		for(int i = 0; i < buffersize.length; i++){
			buffersize[buffersize.length - 1 - i] = response[i + 1];
		}
		int gain = ByteBuffer.wrap(buffersize).getShort();
		
		double[][] result = new double[1][];
		result[0] = new double[1];
		
		result[0][0] = (double)gain;

		return result;
	}
	
	private double[][] runBurnSettings(int burnMode) throws p3AppManagerException
	{
		double[][] result = new double[1][];

		if(burnMode == 3)
		{
			for(burnMode=0; burnMode<3; burnMode++)
			{
				RequestPacket rp = new RequestPacket();

				rp.operation = 7 + burnMode;

				{
					rp.scanTime = 0;
					rp.apodizationSel = 0;
					rp.zeroPadding = 0;
					rp.commonWaveNum = 0; 
					rp.resolution = 0;
					rp.mode = 0;
					rp.opticalGain = 0;
				}

				try {
					sendPacket(rp);
				} catch (Exception e) {
					throw new p3AppManagerException( "Communication error. ", 400);
				}

				byte[] response;
				try {
					response = receivePacket();
				} catch (Exception e) {
					throw new p3AppManagerException( "Communication error. ", 400);
				}

				if(response[0] != (byte)0)
					throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));


				result[0] = new double[1];

				result[0][0] = (double)response[0];
			}
			
		}
		else
		{
			RequestPacket rp = new RequestPacket();

			rp.operation = 7 + burnMode;

			{
				rp.scanTime = 0;
				rp.apodizationSel = 0;
				rp.zeroPadding = 0;
				rp.commonWaveNum = 0; 
				rp.resolution = 0;
				rp.mode = 0;
				rp.opticalGain = 0;
			}

			try {
				sendPacket(rp);
			} catch (Exception e) {
				throw new p3AppManagerException( "Communication error. ", 400);
			}

			byte[] response;
			try {
				response = receivePacket();
			} catch (Exception e) {
				throw new p3AppManagerException( "Communication error. ", 400);
			}

			if(response[0] != (byte)0)
				throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));


			result[0] = new double[1];

			result[0][0] = (double)response[0];

		}
		return result;
	}

	private double[][] runSelfCorr(int runTime, int apodizationIndex, int zeroPadding, int gainIndex, int commonWaveNum) throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 10;
		
		{
			rp.scanTime = runTime;
			rp.apodizationSel = apodizationIndex;
			rp.zeroPadding = zeroPadding;
			rp.commonWaveNum = commonWaveNum; 
			rp.resolution = 0;
		    rp.mode = 0;
		    rp.opticalGain = gainIndex;
		}

		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
		
		double[][] result = new double[1][];
		result[0] = new double[1];
		
		result[0][0] = (double)response[0];

		return result;
	}
	
	private double[][] runWavelengthCorrBG(int runTime, int apodizationIndex, int zeroPadding, int gainIndex, int commonWaveNum) throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 11;
		
		{
			rp.scanTime = runTime;
			rp.apodizationSel = apodizationIndex;
			rp.zeroPadding = zeroPadding;
			rp.commonWaveNum = commonWaveNum; 
			rp.resolution = 0;
		    rp.mode = 0;
		    rp.opticalGain = gainIndex;
		}

		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
		
		double[][] result = new double[1][];
		result[0] = new double[1];
		
		result[0][0] = (double)response[0];

		return result;
	}
	
	private double[][] runWavelengthCorr(int runTime, int apodizationIndex, int zeroPadding, int gainIndex, int commonWaveNum, int[] calibrationWells) throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 12;
		
		{
			rp.scanTime = runTime;
			rp.apodizationSel = apodizationIndex;
			rp.zeroPadding = zeroPadding;
			rp.commonWaveNum = commonWaveNum; 
			rp.resolution = 0;
		    rp.mode = 0;
		    rp.opticalGain = gainIndex;
		    
		    for(int i=0; i < (rp.calibrationWells.length > calibrationWells.length? calibrationWells.length : rp.calibrationWells.length); i++)
		    	rp.calibrationWells[i] = calibrationWells[i];
		}

		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
		
		double[][] result = new double[1][];
		result[0] = new double[1];
		
		result[0][0] = (double)response[0];

		return result;
	}
	
	private double[][] runRestoreDefault() throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 13;
		
		{
			rp.scanTime = 0;
			rp.apodizationSel = 0;
			rp.zeroPadding = 0;
			rp.commonWaveNum = 0; 
			rp.resolution = 0;
		    rp.mode = 0;
		    rp.opticalGain = 0;
		}

		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
		
		double[][] result = new double[1][];
		result[0] = new double[1];
		
		result[0][0] = (double)response[0];

		return result;
	}

	private double[][] sleepAction() throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 15;
		
		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
		
		double[][] result = new double[1][];
		result[0] = new double[1];
		
		result[0][0] = (double)response[0];

		return result;
	}
	
	private double[][] wakeUpAction() throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 16;
		
		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
		
		double[][] result = new double[1][];
		result[0] = new double[1];
		
		result[0][0] = (double)response[0];

		return result;
	}
	
	private double[][] powerOff() throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 17;
		
		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
		
		double[][] result = new double[1][];
		result[0] = new double[1];
		
		result[0][0] = (double)response[0];

		return result;
	}
	
	private double[][] powerOn() throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 18;
		
		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
		
		double[][] result = new double[1][];
		result[0] = new double[1];
		
		result[0][0] = (double)response[0];

		return result;
	}
	
	private void setOpticalSettings(int gainOption) throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 27;
		
		{
			rp.scanTime = 0;
			rp.apodizationSel = 0;
			rp.zeroPadding = 0;
			rp.commonWaveNum = 0; 
			rp.resolution = 0;
		    rp.mode = 0;
		    rp.opticalGain = 0;
		    rp.calibrationWells[0] = gainOption;
		}

		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
	}
	
	
	private void SourceSettings(int t1, int t2_c1, int t2_c2, int t2_tmax, int delta_t, int lamps_count, int lamp_sel) throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 22;
		
		{
			rp.scanTime = 0;
			rp.apodizationSel = 0;
			rp.zeroPadding = 0;
			rp.commonWaveNum = 0; 
			rp.resolution = 0;
		    rp.opticalGain = 0;

		    rp.calibrationWells[0] = (lamp_sel * 256) + lamps_count;
		    rp.calibrationWells[1] = (delta_t * 256) + t1;
		    rp.calibrationWells[2] = (t2_tmax * 65536) + (t2_c2 * 256) + t2_c1;
		}

		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] response;
		try {
			response = receivePacket();
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		if(response[0] != (byte)0)
			throw new p3AppManagerException("DVK error. ", (int)(response[0] + 200));
	}
	
	private byte[] checkBoard() throws p3AppManagerException
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 2;	
		
		try {
			sendPacket(rp);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}
		
		byte[] buffer;
		try {
			buffer = receivePacket(1000);
		} catch (Exception e) {
			throw new p3AppManagerException( "Communication error. ", 400);
		}

		return buffer;
	}


	/*!
	 * This function Reads the ID burned on ROM
	 * @return String of Board ID
	 * @throws TAIFException
	 * @throws InterruptedException 
	 */
	public String readROMID() throws Exception
	{
		RequestPacket rp = new RequestPacket();
		rp.operation = 1;
		sendPacket(rp);

		
		try{
			byte[] buffer = receivePacket();
			int nullIndex = 0;
			for(nullIndex = 0; nullIndex < buffer.length && buffer[nullIndex] != 0; nullIndex++);
			
			byte[] trimmedBuff = new byte[nullIndex];
			System.arraycopy(buffer, 0, trimmedBuff, 0, trimmedBuff.length);

			return new String(trimmedBuff);
		}
		catch(Exception e)
		{
			throw e;
		}
	}
	
	public String[] readSWVersion()
	{
		String[] output = new String[2];
		RequestPacket rp = new RequestPacket();
		rp.operation = 14;
		try{
			sendPacket(rp);
		}
		catch(Exception e)
		{
			output[0] = "0.1";
			output[1] = "0.1";
			return output;
		}	
		
		try{
			byte[] buffer = receivePacket();
			byte[] sub_buffer = new byte[4];
			
			for(int i = 0; i < sub_buffer.length; i++){
				sub_buffer[sub_buffer.length - 1 - i] = buffer[i];
			}
			String DVKVersion = Integer.valueOf(ByteBuffer.wrap(sub_buffer).getInt()).toString();
			
			for(int i = 4; i < sub_buffer.length * 2; i++){
				sub_buffer[sub_buffer.length - 1 - i + 4] = buffer[i];
			}
			String piVersion = Integer.valueOf(ByteBuffer.wrap(sub_buffer).getInt()).toString();
			
			output[0] = DVKVersion.substring(0, 6) + "." + DVKVersion.substring(6, 10);
			output[1] = piVersion.substring(0, 6) + "." + piVersion.substring(6, 10);
			
			return output;
		}
		catch(Exception e)
		{
			output[0] = "0.2";
			output[1] = "0.2";
			return output;
		}
	}

}
