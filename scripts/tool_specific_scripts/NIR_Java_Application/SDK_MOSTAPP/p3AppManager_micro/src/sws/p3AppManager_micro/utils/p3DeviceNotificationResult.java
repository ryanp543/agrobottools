/*!
 *  file   p2DeviceNotificationResult.java
 *
 *  date   Aug 19, 2014
 *
 *  author Ahmad Fadeel
 *
 *  brief  This class has the actions defined for the p2AppManager package
 *
 *  copyright
 *  Copyright (c) 2004-2014. Si-Ware Systems. All Rights Reserved.
 *
 *                PROPRIETARY INFORMATION
 *
 *  This file is CONFIDENTIAL and PROPRIETARY and is for the use of
 *  Si-Ware Systems personnel only. Permission to use it otherwise
 *  must be explicitly obtained from Si-Ware Systems.
 */
package sws.p3AppManager_micro.utils;


import sws.p3AppManager_micro.utils.p3Enumerations.p3AppManagerStatus;
import sws.p3AppManager_micro.utils.p3Enumerations.p3DeviceAction;

public class p3DeviceNotificationResult {
	private p3DeviceAction action;
	private String deviceId;

	private p3AppManagerStatus status;

	public p3DeviceAction getAction() {
		return action;
	}

	public void setAction(p3DeviceAction action) {
		this.action = action;
	}



	public String getDeviceId() {
		return deviceId;
	}

	public void setDeviceId(String deviceId) {
		this.deviceId = deviceId;
	}

	public p3AppManagerStatus getStatus() {
		return status;
	}

	public void setStatus(p3AppManagerStatus status) {
		this.status = status;
	}

}
