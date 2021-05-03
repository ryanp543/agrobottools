package sws.p3AppManager_micro.jobs;

import java.util.concurrent.Future;

import sws.p3AppManager_micro.jobs.p3AbstractJob;
import sws.p3AppManager_micro.jobs.p3AppManagerThreadPool;
import sws.p3AppManager_micro.utils.p3AppManagerException;
import sws.p3AppManager_micro.utils.p3Enumerations.p3AppManagerStatus;

public class p3JobManager {

	private static p3AppManagerThreadPool appManagerThreadPool = p3AppManagerThreadPool.getInstance();

	private p3JobManager() {

	}

	public static <T> Future<T> submit(p3AbstractJob<T> job) {
		try {

			Future<T> future = appManagerThreadPool.submit(job);
			return future;
		} catch (Exception e) {

			throw new p3AppManagerException("Failed to sumbit current job ", p3AppManagerStatus.THREADING_ERROR.getNumVal());

		}

	}

}
