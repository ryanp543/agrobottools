module neoSpectraTool {
	exports sws.p3AppManager_micro.configuration;
	exports sws.spectromost.jfreechart;
	exports sws.spectromost;
	exports sws.p3AppManager_micro.devices;
	exports sws.p3AppManager_micro.utils;
	exports sws.p3AppManager_micro.jobs;
	exports sws.p3AppManager_micro;

	requires java.desktop;
	requires jcommon;
	requires jfreechart;
	requires log4j;
	requires miglayout15.swing;
}