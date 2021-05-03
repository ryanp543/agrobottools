package sws.p3AppManager_micro.utils;

public class p3AppManagerNotification {
	private int action;
	private int status;
	private String deviceId;

	public p3AppManagerNotification(int action, int status, String deviceId) {
		this.action = action;
		this.status = status;
		this.deviceId = deviceId;
	}

	public int getAction() {
		return action;
	}

	public void setAction(int action) {
		this.action = action;
	}

	public int getStatus() {
		return status;
	}

	public void setStatus(int status) {
		this.status = status;
	}

	public String getDeviceId() {
		return deviceId;
	}

	public void setDeviceId(String deviceId) {
		this.deviceId = deviceId;
	}



}
