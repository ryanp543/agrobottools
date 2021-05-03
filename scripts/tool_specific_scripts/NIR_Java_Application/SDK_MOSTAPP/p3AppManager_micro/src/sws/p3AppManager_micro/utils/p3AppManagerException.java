package sws.p3AppManager_micro.utils;

public class p3AppManagerException extends RuntimeException {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private int errorCode = 0;

	public p3AppManagerException(String message, int errorCode) {
		super(message);
		this.errorCode = errorCode;
	}

	public int getErrorCode() {
		return this.errorCode;
	}

}
