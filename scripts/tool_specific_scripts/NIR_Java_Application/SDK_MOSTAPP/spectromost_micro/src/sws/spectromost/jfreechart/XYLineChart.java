/*!
 *  @file   XYLineChart.java
 *
 *  @date   May 25, 2014
 *
 *  @author root
 *
 *  @brief  This class is responsible for creating XYLine Chart
 *
 *  @copyright
 *  Copyright (c) 2004-2014. Si-Ware Systems. All Rights Reserved.
 *
 *                PROPRIETARY INFORMATION
 *
 *  This file is CONFIDENTIAL and PROPRIETARY and is for the use of
 *  Si-Ware Systems personnel only. Permission to use it otherwise
 *  must be explicitly obtained from Si-Ware Systems.
 */
package sws.spectromost.jfreechart;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.LegendItem;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.event.ChartChangeEvent;
import org.jfree.chart.event.ChartChangeListener;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.SamplingXYLineRenderer;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.xy.DefaultXYDataset;
import org.jfree.data.xy.XYDataset;

import java.awt.Color;
import java.awt.Paint;
import java.awt.geom.Rectangle2D;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class XYLineChart  {

	private ChartPanel chartPanel;
	private DefaultXYDataset dataset;
	
	private double upper,lower;
	private double minXLimit, maxXLimit, minYLimit, maxYLimit;

	private Color[] colors = {
			new Color(0,0,255),
			new Color(255,20,147),
			new Color(255,215,0),
			new Color(0,255,0),
			new Color(0,128,128),
			new Color(106,90,205),
			new Color(210,105,30),
			new Color(154,205,50),
			new Color(0,255,255),
			new Color(255,140,0),
			new Color(139,0,0),
			new Color(124,252,0),
			new Color(138,43,226),
			new Color(105,105,105),
			new Color(102,205,170),
			new Color(255,69,0),
			new Color(0,0,0),
			new Color(106,90,205),
			new Color(255,255,0),
			new Color(50,205,50),
			new Color(25,25,112),
			new Color(255,0,0),
			new Color(128,128,0),
			new Color(139,69,19),
			new Color(148,0,211),
			new Color(0,100,0),
			new Color(47,79,79),
			new Color(178,34,34),
			new Color(30,144,255),
			new Color(0,128,0),
			new Color(255,105,180)};

	public ChartPanel getChartPanel()
	{
		return chartPanel;
	}

	public XYLineChart(String chartTitle, Comparable<String> seriesKey,String xAxisLabel, String yAxisLabel, double [][] data) 
	{
		dataset =  new DefaultXYDataset();
		addSeries(seriesKey, data);
		JFreeChart chart = createXYLineChart(dataset, chartTitle,xAxisLabel,yAxisLabel, 0, 0, 0, 0);
		// we put the chart into a panel
		chartPanel = new ChartPanel(chart);
		
		upper = 0;
		lower = 0;
	}
	
	public XYLineChart(String chartTitle, Comparable<String> seriesKey,String xAxisLabel, String yAxisLabel, double [][] data, boolean reScale) 
	{
		dataset =  new DefaultXYDataset();
		addSeries(seriesKey, data);
		JFreeChart chart = createXYLineChart(dataset, chartTitle,xAxisLabel,yAxisLabel, 0, 0, 0, 0);
		// we put the chart into a panel
		chartPanel = new ChartPanel(chart);
		if(reScale == true)
		{
			upper = Double.MIN_VALUE;
			lower = Double.MAX_VALUE;
			updateLowerUpper(dataset);
			ReScale();	
		}
	}
	
	public XYLineChart(String chartTitle, Comparable<String> seriesKey,String xAxisLabel, String yAxisLabel, double [][] data,
			double minXLimit, double maxXLimit, double minYLimit, double maxYLimit) 
	{
		dataset =  new DefaultXYDataset();
		addSeries(seriesKey, data);

		this.minXLimit = minXLimit;
		this.maxXLimit = maxXLimit;
		this.minYLimit = minYLimit;
		this.maxYLimit = maxYLimit;
		
		JFreeChart chart = createXYLineChart(dataset, chartTitle,xAxisLabel,yAxisLabel, minXLimit, maxXLimit, minYLimit, maxYLimit);
		
		chart.addChangeListener(new ChartChangeListener() {
			
			@Override
			public void chartChanged(ChartChangeEvent arg0) {
//				System.out.println("change");
				if(arg0.getChart() != null)
					chartChangeHandeling(arg0);
			}
		});
		
		// we put the chart into a panel
		chartPanel = new ChartPanel(chart);
		
		upper = maxYLimit;
		lower = minYLimit;
	}
	
	private void chartChangeHandeling(ChartChangeEvent arg0)
	{
		if(dataset.getSeriesCount() == 1 &&
				   dataset.getItemCount(0) == 1)
		{
			if(arg0.getChart().getXYPlot().getDomainAxis().getRange().getLowerBound() != minXLimit &&
			   arg0.getChart().getXYPlot().getDomainAxis().getRange().getUpperBound() != maxXLimit )
				arg0.getChart().getXYPlot().getDomainAxis().setRange(minXLimit, maxXLimit);
			if(arg0.getChart().getXYPlot().getRangeAxis().getRange().getLowerBound() != minYLimit &&
			   arg0.getChart().getXYPlot().getRangeAxis().getRange().getUpperBound() != maxYLimit )
				arg0.getChart().getXYPlot().getRangeAxis().setRange(minYLimit, maxYLimit);	
		}
	}
	
	
	public void addSeries(Comparable<String> seriesKey, double [][] data) {
		addSeries(seriesKey, data, false);
	}

	@SuppressWarnings("unchecked")
	public void addSeries(Comparable<String> seriesKey, double [][] data, boolean reScale) {
		DefaultXYDataset tempDataSet = new DefaultXYDataset(); 
		tempDataSet.addSeries(seriesKey, data);
		
		for(int i=0;i<dataset.getSeriesCount();i++)
		{
			Comparable<String> key =  dataset.getSeriesKey(i);
			if(key.equals(seriesKey))
				continue;
			tempDataSet.addSeries(key, getDataOfSeries(dataset,i));
		}
		
		clearDataSet();
		for(int i=0;i<tempDataSet.getSeriesCount();i++)
		{
			dataset.addSeries(tempDataSet.getSeriesKey(i), getDataOfSeries(tempDataSet,i));
		}
		
		
		if(reScale == true)
		{
			updateLowerUpper(dataset);
			ReScale();
		}
	}

	private double[][] getDataOfSeries(DefaultXYDataset dataset,int index)
	{
		double[][] data = new double[2][];
		data[0] = new double[dataset.getItemCount(index)];
		data[1] = new double[dataset.getItemCount(index)];
		for(int j=0;j<dataset.getItemCount(index);j++)
		{
			data[0][j] = dataset.getXValue(index, j);
			data[1][j] = dataset.getYValue(index, j);
		}
		return data;
	}
	
	private void clearDataSet()
	{
		while(dataset.getSeriesCount()!=0)
		{
			dataset.removeSeries(dataset.getSeriesKey(0));
		}
	}
	
	private void updateLowerUpper(DefaultXYDataset dataset)
	{
		upper = Double.MIN_VALUE;
		lower = Double.MAX_VALUE;
		
		for(int i=0;i<dataset.getSeriesCount();i++)
		{
			double[][] data = getDataOfSeries(dataset,i);

			double max_y = maxDouble(data[1]);
			double min_y = minDouble(data[1]);

			if(max_y>upper)
			{
				upper = max_y;
				upper += upper * 0.02;
			}
			if(min_y<lower)
			{
				lower = min_y;
				lower -= upper * 0.02;
			}
		}
	}
	
	private void ReScale()
	{
		JFreeChart chart = chartPanel.getChart();
		XYPlot xyPlot = chart.getXYPlot();
		
		ValueAxis axis = xyPlot.getRangeAxis();
		axis.setLowerBound(lower);
		axis.setUpperBound(upper);
	}
	
	private JFreeChart createXYLineChart(XYDataset dataset, String title,String xAxisLabel, String yAxisLabel , 
			double minXLimit, double maxXLimit, double minYLimit, double maxYLimit) 
	{
		JFreeChart chart = ChartFactory.createXYLineChart( title, xAxisLabel, yAxisLabel, dataset);
		XYPlot xyPlot = (XYPlot) chart.getPlot();

		//Change grid background and grid lines colors
		xyPlot.setBackgroundPaint(Color.white);
		xyPlot.setRangeGridlinePaint(Color.black);
		xyPlot.setDomainGridlinePaint(Color.black);

		if( minXLimit != 0)
		{
			ValueAxis axis = xyPlot.getDomainAxis();
			axis.setLowerBound( minXLimit);
		}

		if(maxXLimit != 0)
		{
			ValueAxis axis = xyPlot.getDomainAxis();
			axis.setUpperBound( maxXLimit);
		}

		if( minYLimit != 0)
		{
			ValueAxis axis = xyPlot.getRangeAxis();
			axis.setLowerBound( minYLimit);
		}

		if( maxYLimit != 0)
		{
			ValueAxis axis = xyPlot.getRangeAxis();
			axis.setUpperBound( maxYLimit);
		}

		XYItemRenderer r = new SamplingXYLineRenderer() {
			private static final long serialVersionUID = 1L;

			public Paint getSeriesPaint(int series)
			{
				XYDataset dataset = getPlot().getDataset();

				String label = dataset.getSeriesKey(series).toString();
				Matcher matcher = Pattern.compile("\\d+").matcher(label);

				int colorIndex = 0;
				if(matcher.find()) {
					colorIndex = Integer.parseInt(matcher.group());
					colorIndex--;
				}
				else if(label.contains("Neg"))
				{
					colorIndex = 0;
				}
				else if(label.contains("Pos"))
				{
					colorIndex = 1;
				}
				colorIndex = colorIndex % colors.length;
				return colors[colorIndex];
			}

			public LegendItem getLegendItem(int datasetIndex, int series) 
			{
				XYDataset dataset = getPlot().getDataset(datasetIndex);

				String label = dataset.getSeriesKey(series).toString();
				LegendItem legendItem = new LegendItem(label, lookupSeriesPaint(series));
				legendItem.setLine(new Rectangle2D.Double( 0.0 , 0.0 , 5.0, 5.0) );  //setLine takes a Shape, not just Lines so you can pass any Shape to it...
				return legendItem;
			}

		};
		xyPlot.setRenderer(r);

		return chart;

	}
	public void removeSeries(String string) {
		this.removeSeries(string);

	}
	
	private double maxDouble(double[] arr)
	{
		double max = arr[0];
		
		for(int i=1;i<arr.length;i++)
		{
			if(max < arr[i])
				max = arr[i];
		}
		
		return max;
	}
	
	private double minDouble(double[] arr)
	{
		double min = arr[0];
		
		for(int i=1;i<arr.length;i++)
		{
			if(min > arr[i])
				min = arr[i];
		}
		
		return min;
	}
	
	public double[][] getGraphData(String seriesKey)
	{
		return getDataOfSeries(dataset, dataset.indexOf(seriesKey));
	}
	
	public String[] getSeriesKeys()
	{
		String[] result = new String[dataset.getSeriesCount()];
		
		for(int i = 0; i < dataset.getSeriesCount(); i++)
			result[i] = dataset.getSeriesKey(i).toString();
		
		return result;
	}
}