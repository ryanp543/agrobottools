package sws.p3AppManager_micro.jobs;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class p3AppManagerThreadPool extends ThreadPoolExecutor {

	static final int defaultCorePoolSize = 3;
	static final int defaultMaximumPoolSize =3;
	static final long defaultKeepAliveTime = 3;
	static final TimeUnit defaultTimeUnit = TimeUnit.MINUTES;
	static final BlockingQueue<Runnable> workQueue = new LinkedBlockingQueue<Runnable>();
	private static p3AppManagerThreadPool instance;

	private p3AppManagerThreadPool() {
		super(defaultCorePoolSize, defaultMaximumPoolSize,
				defaultKeepAliveTime, defaultTimeUnit, workQueue);
	}

	synchronized static p3AppManagerThreadPool getInstance() {
		if (instance == null) {
			instance = new p3AppManagerThreadPool();
		}
		return instance;
	}

	

}
