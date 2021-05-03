package sws.p3AppManager_micro.utils;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.text.MessageFormat;
import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Logger;

public class p3AppManagerUtils {
	private static Logger logger = Logger.getLogger(p3AppManagerUtils.class);

	private p3AppManagerUtils() {
	}

	public static boolean isFilenameValid(String fileName) {
		// Check for illegal characters
		for (char c : fileName.toCharArray()) {
			switch (c) {
			case '/':
			case '\n':
			case '\r':
			case '\t':
			case '\0':
			case '\f':
			case '`':
			case '?':
			case '*':
			case '\\':
			case '<':
			case '>':
			case '|':
			case '\"':
			case ':':
				return false;
			}
		}
		return true;
	}

	// / <summary>
	// / check if directory exists or not and create it if not
	// / </summary>
	// / <param name="path">dir path </param>
	// / <returns> boolean</returns>
	public static boolean createDir(String path) {

		// check out folder exist or not
		File folder = new File(path);
		if (!folder.exists()) {
			return folder.mkdirs();
		}

		return true;
	}

	// / <summary>
	// / check if directory exists or not and delete it if yes
	// / </summary>
	// / <param name="path">dir path </param>
	// / <returns> boolean</returns>
	public static boolean removeDir(String path) {
		// check out folder exist or not
		File folder = new File(path);
		if (folder.exists()) {
			
			String[]entries = folder.list();
			for(String s: entries){
				File currentFile = new File(folder.getPath(),s);
				currentFile.delete();
			}

			return folder.delete();
		}

		return true;
	}
	
	public static boolean removeDir(String path, boolean recursive) {
		// check out folder exist or not
		File folder = new File(path);
		if (folder.exists()) {
			
			String[]entries = folder.list();
			for(String s: entries){
				File currentFile = new File(folder.getPath(),s);
				if(currentFile.isDirectory() && recursive)
					removeDir(currentFile.getPath(),true);
				currentFile.delete();
			}

			return folder.delete();
		}

		return true;
	}
	// / <summary>
	// / check if directory exists or not and create it if not
	// / </summary>
	// / <param name="path">dir path </param>
	// / <returns> boolean</returns>
	public static boolean exist(String filePath) {

		// check out folder exist or not
		File file = new File(filePath);
		return file.exists();

	}

	// / <summary>
	// / getFolderSubFolders
	// / </summary>
	// / <param name="filePath">file path to loads its sub folders </param>
	// / <param name="allLevals">get all level subfolders or only one </param>
	// / <returns> sub folders for one lavel or all subfolders </returns>
	public static String[] getFolderSubFoldersNames(String directoryName, ArrayList<String> folderNames,
			boolean allLevals) {
		File directory = new File(directoryName);
		// get all the files from a directory
		File[] fList = directory.listFiles();
		for (File file : fList) {
			if (file.isDirectory() && !file.isHidden()) {
				folderNames.add(file.getName());
				if (allLevals)
					getFolderSubFoldersNames(file.getAbsolutePath(), folderNames, true);
			}
		}

		return (String[]) folderNames.toArray(new String[folderNames.size()]);
	}

	// / <summary>
	// / getFolderSubFiles
	// / </summary>
	// / <param name="filePath">file path to loads its sub files </param>
	// / <returns> sub files </returns>
	public static String[] getFolderSubFilesNames(String directoryName, ArrayList<String> fileNames) {
		File directory = new File(directoryName);
		// get all the files from a directory
		File[] fList = directory.listFiles();
		for (File file : fList) {
			if (!file.isDirectory() && !file.isHidden()) {
				fileNames.add(file.getName());
			}
		}

		return (String[]) fileNames.toArray(new String[fileNames.size()]);
	}

	// / <summary>
	// / format String
	// / </summary>
	// / <param name="template">template to use in format </param>
	// / <param name="params">parameters to replace placeholders </param>
	// / <returns> formatted string</returns>
	public static boolean isEmptyString(String str) {

		return (str == null || "".equals(str)) ? true : false;
	}

	// / <summary>
	// / format String
	// / </summary>
	// / <param name="template">template to use in format </param>
	// / <param name="params">parameters to replace placeholders </param>
	// / <returns> formatted string</returns>
	public static String formatString(String template, Object... params) {
		return MessageFormat.format(template, params);
	}

	// / <summary>
	// / read parameters file it's path is specified and parse its values
	// / </summary>
	// / <param name="strFilePath">the path of the file you want to read</param>
	// / <returns>the list of values (parameters) found in the file</returns>
	public static double[] loadParamFile(String strFilePath) {

		// System.out.println("P2 :::: Loading param config path: " +
		// strFilePath);
		ArrayList<Double> lstReturn = new ArrayList<Double>();
		BufferedReader reader = null;
		try {
			reader = new BufferedReader(new FileReader(strFilePath));

			String line = null;
			while ((line = reader.readLine()) != null) {
				String[] strLineTokens = line.split(":");

				if (strLineTokens.length == 2) {
					lstReturn.add(Double.parseDouble(strLineTokens[1]));
				}
			}

		} catch (Exception ex) {
			logger.error(ex.getMessage());
			return null;
		} finally {
			try {
				reader.close();
			} catch (Exception ex) {
				logger.error(ex.getMessage());
			}
		}

		double[] arrayToReturn = new double[lstReturn.size()];
		for (int y = 0; y < arrayToReturn.length; y++) {
			arrayToReturn[y] = lstReturn.get(y);
		}
		return arrayToReturn;
	}

	// / <summary>
	// / read streaming (just numbers .. no text) data from file specified
	// / </summary>
	// / <param name="strFilePath">the path of the file you want to read</param>
	// / <returns>list of values</returns>
	public static double[] loadRawDataFile(String strFilePath) {

		ArrayList<Double> lstReturn = new ArrayList<Double>();
		BufferedReader reader = null;
		try {
			reader = new BufferedReader(new FileReader(strFilePath));

			String line = null;
			while ((line = reader.readLine()) != null) {
				lstReturn.add(Double.parseDouble(line));
			}

		} catch (Exception ex) {
			logger.error(ex.getMessage());
			return null;
		} finally {
			try {
				reader.close();
			} catch (Exception ex) {
				logger.error(ex.getMessage());
			}
		}

		double[] arrayToReturn = new double[lstReturn.size()];
		for (int y = 0; y < arrayToReturn.length; y++) {
			arrayToReturn[y] = lstReturn.get(y);
		}
		return arrayToReturn;
	}
	
	public static int[] loadIntDataFile(String strFilePath) {

		ArrayList<Integer> lstReturn = new ArrayList<Integer>();
		BufferedReader reader = null;
		try {
			reader = new BufferedReader(new FileReader(strFilePath));

			String line = null;
			while ((line = reader.readLine()) != null) {
				lstReturn.add(Integer.parseInt(line));
			}

		} catch (Exception ex) {
			logger.error(ex.getMessage());
			return null;
		} finally {
			try {
				reader.close();
			} catch (Exception ex) {
				logger.error(ex.getMessage());
			}
		}

		int[] arrayToReturn = new int[lstReturn.size()];
		for (int y = 0; y < arrayToReturn.length; y++) {
			arrayToReturn[y] = lstReturn.get(y);
		}
		return arrayToReturn;
	}

	public static long[] loadLongDataFile(String strFilePath) {

		ArrayList<Long> lstReturn = new ArrayList<Long>();
		BufferedReader reader = null;
		try {
			reader = new BufferedReader(new FileReader(strFilePath));

			String line = null;
			while ((line = reader.readLine()) != null) {
				lstReturn.add(Long.parseLong(line));
			}

		} catch (Exception ex) {
			logger.error(ex.getMessage());
			return null;
		} finally {
			try {
				reader.close();
			} catch (Exception ex) {
				logger.error(ex.getMessage());
			}
		}

		long[] arrayToReturn = new long[lstReturn.size()];
		for (int y = 0; y < arrayToReturn.length; y++) {
			arrayToReturn[y] = lstReturn.get(y);
		}
		return arrayToReturn;
	}
	// / <summary>
	// / convert multiples 1D arrays of spectroscopy or interferogram or cap and
	// current processing into one 1D array (length1 array1 length2 array2 ...)
	// / </summary>
	// / <param name="type"></param>
	// / <param name="data"></param>
	// / <returns>The newly converted array</returns>
	public static double[] concatenateMultipleArraysIntoOne(double[]... arrays) {

		try {
			int lengthOFFinalArray = 0;
			double[] FinalArray;

			for (double[] array : arrays) {
				lengthOFFinalArray += array.length + 1;
			}
			FinalArray = new double[lengthOFFinalArray];

			int index = 0;
			for (double[] array : arrays) {
				FinalArray[index] = array.length;
				System.arraycopy(array, 0, FinalArray, index + 1, (int) FinalArray[index]);
				index += array.length + 1;
			}

			return FinalArray;
		} catch (Exception ex) {
			logger.error(ex.getMessage());

			return null;
		}

	}

	// / <summary>
	// / this method used to decode byte array data depend on its data type
	// / </summary>
	// / <returns>list of data </returns>
	public static List<String> decodeBytesToValues(int type, byte[] data) {
		List<String> decodedData = new ArrayList<>();
		switch (type) {
		case 0:

			// loop on data array to parse every 8 bytes to one double value.
			for (int i = 0; i < data.length; i = i + 8) {

				decodedData.add(Double.valueOf(p3NumberConverter.toDouble(data, i)).toString());
				// new BigInteger(Arrays.copyOfRange(data, i, i +
				// 8)).doubleValue()).toString());
			}
			return decodedData;
		case 1:

			// loop on data array to parse every 4 bytes to one int value.
			for (int i = 0; i < data.length; i = i + 4) {

				decodedData.add(Integer.valueOf(p3NumberConverter.ToInt(data, i)).toString());
			}

			return decodedData;
		case 2:

			decodedData.add(new String(data, StandardCharsets.US_ASCII));
			return decodedData;

		case 3:
			// loop on data array to parse every 4 bytes to one float value.
			for (int i = 0; i < data.length; i = i + 4) {

				decodedData.add(Float.valueOf(p3NumberConverter.ToSingle(data, i)).toString());
			}
			return decodedData;
		default:
			logger.error("invalid data type ");

		}
		return null;
	}

	// / <summary>
	// / read parameters file it's path is specified and parse its values
	// / </summary>
	// / <param name="strFilePath">the path of the file you want to read</param>
	// / <returns>the list of values (parameters) found in the file</returns>
	public static String[] readStringfile(String strFilePath) {

		ArrayList<String> lstReturn = new ArrayList<String>();
		BufferedReader reader = null;
		try {
			reader = new BufferedReader(new FileReader(strFilePath));

			String line = null;
			while ((line = reader.readLine()) != null) {
				lstReturn.add(line);
			}

		} catch (Exception ex) {
			logger.error(ex.getMessage());
			return null;
		} finally {
			try {
				reader.close();
			} catch (Exception ex) {
				logger.error(ex.getMessage());
			}
		}

		return lstReturn.toArray(new String[lstReturn.size()]);
	}

	public static boolean writeFileOfArray(int[] data, String path, String c) {

		Writer writer = null;
		boolean writeSperator = c != null ? true : false;
		try {

			writer = new BufferedWriter(new FileWriter((path)));
			for (int d : data) {
				writer.write(Integer.toString(d));
				if (writeSperator)
					writer.write(c);
			}
			writer.close();

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

	public static boolean writeFileOfArray(double[] data, String path, String c) {

		Writer writer = null;
		boolean writeSperator = c != null ? true : false;
		try {

			writer = new BufferedWriter(new FileWriter((path)));
			for (double d : data) {
				writer.write(Double.toString(d));
				if (writeSperator)
					writer.write(c);
			}
			writer.close();

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

	// / <summary>
	// / write the contents of an array into a specific file with separator
	// character.
	// / </summary>
	// / <param name="data">array of doubles you want to write in
	// the file</param>
	// / <param name="path">path of the file you want to write</param>
	// / <param name="c">separator char</param>
	public static boolean writeFileOfArray(String[] data, String path, String c) {

		Writer writer = null;
		boolean writeSperator = c != null ? true : false;
		try {

			writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(path), "utf-8"));
			for (String item : data) {
				writer.write(item);
				if (writeSperator)
					writer.write(c);
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

}
