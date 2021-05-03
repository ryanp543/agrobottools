package sws.p3AppManager_micro.jobs;

import java.util.concurrent.Callable;


public abstract class p3AbstractJob<T> implements Callable<T> {

	@Override
	public T call() throws Exception {
		return execute();
	}

	protected abstract T execute();

}
