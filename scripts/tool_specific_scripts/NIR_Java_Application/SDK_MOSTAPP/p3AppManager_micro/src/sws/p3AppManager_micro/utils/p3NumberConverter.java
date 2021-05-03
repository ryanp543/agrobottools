package sws.p3AppManager_micro.utils;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class p3NumberConverter {

	//This function converts 4 bytes to double
	public static double toDouble(byte[] b, int offset) {
		try {
			ByteBuffer buf = ByteBuffer.wrap(b, offset, 8);
			buf.order(ByteOrder.nativeOrder());
			return buf.getDouble();
		} catch (Exception e) {
			return 0;
		}
	}

	//This function converts 4 bytes to single
	public static float ToSingle(byte[] b, int offset) {
		try {
			ByteBuffer buf = ByteBuffer.wrap(b, offset, 4);
			buf.order(ByteOrder.nativeOrder());
			return buf.getFloat();
		} catch (Exception e) {
			return 0;
		}
	}

	//This function converts 4 bytes to short
	public static short ToShort(byte[] b, int offset) {
		try {
			ByteBuffer buf = ByteBuffer.wrap(b, offset, 2);
			buf.order(ByteOrder.nativeOrder());
			return buf.getShort();
		} catch (Exception e) {
			return 0;
		}
	}

	//This function converts 4 bytes to int
	public static int ToInt(byte[] b, int offset) {
		try {
			ByteBuffer buf = ByteBuffer.wrap(b, offset, 4);
			buf.order(ByteOrder.nativeOrder());
			return buf.getInt();
		} catch (Exception e) {
			return 0;
		}

	}

	//This function converts 1 byte to int
	public static int byteToNo(byte[] b, int offset) {
		try {
			return (int)b[0] ;
		} catch (Exception e) {
			return 0;
		}

	}

}
