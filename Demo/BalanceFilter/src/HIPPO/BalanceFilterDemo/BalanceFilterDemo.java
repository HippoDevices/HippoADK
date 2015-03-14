/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package HIPPO.BalanceFilterDemo;

import android.app.Activity;
import android.content.Context;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory.Options;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;

/**
 * <h3>Application that displays the values of the acceleration sensor graphically.</h3>

<p>This demonstrates the {@link android.hardware.SensorManager android.hardware.SensorManager} class.

<h4>Demo</h4>
OS / Sensors
 
<h4>Source files</h4>
 * <table class="LinkTable">
 *         <tr>
 *             <td >src/com.example.android.apis/os/Sensors.java</td>
 *             <td >Sensors</td>
 *         </tr>
 * </table> 
 */
public class BalanceFilterDemo extends Activity {
    private SensorManager mSensorManager;
    private GraphView mGraphView;
	private String TAG = "HIPPO";

	private long Current_Time=0, Early_Time=0, Use_Time=0;
	private float Q_angle=0.005f, Q_gyro=0.003f, R_angle=0.5f, dt=0.02f;
	private float x, y, z;
	private float q_bias=0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
	private float P[][] = { { 1, 0 }, { 0, 1 } };
	private float Pdot[] ={0,0,0,0};
	//private float Gyroscope[] ={0,0,0,0,0,0,0,0,0,0};
	private float Angle,Angle_h,Angle_Dot,Angle_Accelerometer,Sum_Measure_Gyroscope,Sum_Gyroscope,Measure_Gyroscope,Gyroscope; 		//外部需要引用的变量
	private char C_0 = 1;
	private int  Counter_Gyroscope = 0;
	private int  Screen_Height = 0;
	
	
    private class GraphView extends View implements SensorEventListener
    {
        private Bitmap  mBitmap;
        private Bitmap  LogoBitmap;
        private Paint   mPaint = new Paint();
        private Canvas  mCanvas = new Canvas();
        private Path    mPath = new Path();
        private RectF   mRect = new RectF();
        private float   mLastValues[] = new float[3*2];
        private float   mOrientationValues[] = new float[3];
        private int     mColors[] = new int[3*2];
        private float   mLastX;
        private float   mScale[] = new float[2];
        private float   mYOffset;
        private float   mMaxX;
        private float   mSpeed = 1.0f;
        private float   mWidth;
        private float   mHeight;
        
        
        
        public GraphView(Context context) {
            super(context);
            mColors[0] = Color.argb(192, 255, 64, 64);
            mColors[1] = Color.argb(192, 64, 128, 64);
            mColors[2] = Color.argb(192, 64, 64, 255);
            mColors[3] = Color.argb(192, 64, 255, 255);
            mColors[4] = Color.argb(192, 128, 64, 128);
            mColors[5] = Color.argb(192, 255, 255, 64);

            mPaint.setFlags(Paint.ANTI_ALIAS_FLAG);
            mRect.set(-0.5f, -0.5f, 0.5f, 0.5f);
            mPath.arcTo(mRect, 0, 180);
        }
        
        @Override
        protected void onSizeChanged(int w, int h, int oldw, int oldh) {
            mBitmap = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
            mCanvas.setBitmap(mBitmap);
            mCanvas.drawColor(0xFFFFFFFF);
            Screen_Height=h;
            mYOffset = h * 0.5f;
            mScale[0] = - h/60;
            mWidth = w;
            mHeight = h;
            if (mWidth < mHeight) {
                mMaxX = w;
            } else {
                mMaxX = w-50;
            }
            mLastX = mMaxX;
            super.onSizeChanged(w, h, oldw, oldh);
        }

        @Override
        protected void onDraw(Canvas canvas) {
            synchronized (this) {
                if (mBitmap != null) {
                    final Paint paint = mPaint;
                    final Path path = mPath;
                    final int outer = 0xFFC0C0C0;
                    final int inner = 0xFFff7010;

                    if (mLastX >= mMaxX) {
                        mLastX = 0;
                        final Canvas cavas = mCanvas;
                        final float yoffset = mYOffset;
                        final float maxx = mMaxX;
                        final float oneG = SensorManager.STANDARD_GRAVITY * mScale[0];
                        paint.setColor(0xFFAAAAAA);
                        cavas.drawColor(0xFFFFFFFF);
                        //Bitmap ball = BitmapFactory.decodeResource(getResources(), R.drawable.logo);
                        //LogoBitmap = Bitmap.createScaledBitmap(ball, 5, 20, true);
                        //canvas.drawBitmap( LogoBitmap, 20, 20, paint );//插入图标
                        //Bitmap bmp=BitmapFactory.decodeResource(res, R.drawable.pic);
                        paint.setColor(mColors[0]);
                        paint.setTextSize(100);
                        canvas.drawText( "www.nwtel.cn", 50, 50, paint );//插入图标
                        cavas.drawLine(0, Screen_Height*0.1f, maxx,Screen_Height*0.1f, paint);
                        cavas.drawLine(0, Screen_Height*0.3f, maxx, Screen_Height*0.3f, paint);
                        cavas.drawLine(0, Screen_Height*0.5f, maxx, Screen_Height*0.5f, paint);
                        cavas.drawLine(0, Screen_Height*0.7f, maxx, Screen_Height*0.7f, paint);
                        cavas.drawLine(0, Screen_Height*0.9f, maxx, Screen_Height*0.9f, paint);
                    }
                    canvas.drawBitmap(mBitmap, 0, 0, null);

                    float[] values = mOrientationValues;
                    if (mWidth < mHeight) {
                        float w0 = mWidth * 0.333333f;
                        float w  = w0 - 32;
                        float x = w0*0.5f;
                        for (int i=0 ; i<3 ; i++) {
                            canvas.save(Canvas.MATRIX_SAVE_FLAG);
                            canvas.translate(x, w*0.5f + 4.0f);
                            canvas.save(Canvas.MATRIX_SAVE_FLAG);
                            paint.setColor(outer);
                            canvas.scale(w, w);
                            canvas.drawOval(mRect, paint);
                            canvas.restore();
                            canvas.scale(w-5, w-5);
                            if(i==0) {
                            	canvas.rotate(-Angle_Accelerometer);
                            paint.setColor(mColors[0]);
                            }
                            if(i==1) {
                            	canvas.rotate(-Angle);
                            paint.setColor(mColors[2]);
                            }
                            if(i==2){ 
                            	canvas.rotate(-Sum_Measure_Gyroscope);
                            paint.setColor(mColors[3]);
                            }
                            canvas.drawPath(path, paint);
                            canvas.restore();
                            x += w0;
                        }
                    } else {
                        float h0 = mHeight * 0.333333f;
                        float h  = h0 - 32;
                        float y = h0*0.5f;
                        for (int i=0 ; i<3 ; i++) {
                            canvas.save(Canvas.MATRIX_SAVE_FLAG);
                            canvas.translate(mWidth - (h*0.5f + 4.0f), y);
                            canvas.save(Canvas.MATRIX_SAVE_FLAG);
                            paint.setColor(outer);
                            canvas.scale(h, h);
                            canvas.drawOval(mRect, paint);
                            canvas.restore();
                            canvas.scale(h-5, h-5);
                            paint.setColor(inner);
                            if(i==0) {
                            	canvas.rotate(-Angle_Accelerometer);
                            paint.setColor(mColors[0]);
                            }
                            if(i==1) {
                            	canvas.rotate(-Angle);
                            paint.setColor(mColors[2]);
                            }
                            if(i==2) {
                            	canvas.rotate(-Sum_Measure_Gyroscope);
                            paint.setColor(mColors[3]);
                            }
                            canvas.drawPath(path, paint);
                            canvas.restore();
                            y += h0;
                        }
                    }

                }
            }
        }

        public void onSensorChanged(SensorEvent event) {
            synchronized (this) {
                if (mBitmap != null) {
                    final Canvas canvas = mCanvas;
                    final Paint paint = mPaint;
                    if (event.sensor.getType() == Sensor.TYPE_ORIENTATION) {
                        for (int i=0 ; i<3 ; i++) {
                            mOrientationValues[i] = event.values[i];
                        }
                    }
                    if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER){                     
        				x = event.values[SensorManager.DATA_X];
        				y = event.values[SensorManager.DATA_Y];
        				z = event.values[SensorManager.DATA_Z];        				        				
        				 float g = (float)Math.sqrt(z * z + y*y );//手机横放，机器人围绕X轴旋转
        				 float sin = y / g;//手机横放，机器人围绕X轴旋转        				
        				 Angle_Accelerometer = (float) Math.asin(sin) * 57.3f;// 手机横放，机器人围绕X轴旋转
                     	Current_Time=event.timestamp;
                     	//Current_Time= System.currentTimeMillis();
         				Measure_Gyroscope= 57.3f*(event.values[SensorManager.DATA_X]) + 1.5f;//竖放
                      	if(Early_Time==0)
                     		Use_Time=20300000;
                     	else
                     	Use_Time=(Current_Time-Early_Time);
         				Early_Time=Current_Time;
         				dt = (float)Use_Time *(1.0f / 1000000000.0f);
         				Log.e("duzhipeng","dt = " + dt);
        				 
                    }
        			if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {   
                    	Current_Time=event.timestamp;
                    	//Current_Time= System.currentTimeMillis();
        				Measure_Gyroscope= 57.3f*(event.values[SensorManager.DATA_X]) + 1.5f;//竖放
        				
        				//Log.e("duzhipeng","DATA_X = " + Gyroscope);
        				
        				//Angle_h = Angle_h + (((Angle_Accelerometer-Angle_h)*0.5f + Measure_Gyroscope)*0.009f);
        				
        				
        				//dt=0.009f;
        				Sum_Measure_Gyroscope+=Measure_Gyroscope*dt;
        				if(Sum_Measure_Gyroscope<-80)
        					Sum_Measure_Gyroscope=0;
        				Kalman_Filter(Angle_Accelerometer,Measure_Gyroscope);

                        float deltaX = mSpeed;
                        float newX = mLastX + deltaX;
                        final float v = mYOffset + (Angle_Accelerometer) * mScale[0];
                        paint.setColor(mColors[0]);
                        canvas.drawLine(mLastX, mLastValues[0], newX, v, paint);
                        mLastValues[0] = v; 
                        final float v1 = mYOffset + (Angle) * mScale[0];
                        paint.setColor(mColors[2]);
                        canvas.drawLine(mLastX, mLastValues[1], newX, v1, paint);
                        mLastValues[1] = v1; 
                        final float v2 = mYOffset + (Sum_Measure_Gyroscope) * mScale[0];
                        paint.setColor(mColors[3]);
                        canvas.drawLine(mLastX, mLastValues[2], newX, v2, paint);
                        mLastValues[2] = v2;                                                                                            
                            mLastX += mSpeed;    
        			}
                    
                    invalidate();
                }
            }
        }

        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }
    }
    
    /**
     * Initialization of the Activity after it is first created.  Must at least
     * call {@link android.app.Activity#setContentView setContentView()} to
     * describe what is to be displayed in the screen.
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        // Be sure to call the super class.
        super.onCreate(savedInstanceState);
        this.setTitle("BalanceFilterDemo-www.hippodevices.com");
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mGraphView = new GraphView(this);
        setContentView(mGraphView);
    }

    @Override
    protected void onResume() {
        super.onResume();
        mSensorManager.registerListener(mGraphView,
                mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                20000);
        mSensorManager.registerListener(mGraphView,
                mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                10000);
        mSensorManager.registerListener(mGraphView, 
                mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION),
                SensorManager.SENSOR_DELAY_FASTEST);
    }
    
    @Override
    protected void onStop() {
        mSensorManager.unregisterListener(mGraphView);
        super.onStop();
    }
    
	void Kalman_Filter(float angleaccelerometer,float anglegyroscope)		
	{
		Angle += (anglegyroscope - q_bias) * dt;
		Pdot[0] = Q_angle - P[0][1] - P[1][0];
		Pdot[1] = -P[1][1];
		Pdot[2] = -P[1][1];
		Pdot[3] = Q_gyro;

		P[0][0] += Pdot[0] * dt;
		P[0][1] += Pdot[1] * dt;
		P[1][0] += Pdot[2] * dt;
		P[1][1] += Pdot[3] * dt;

		angle_err = angleaccelerometer - Angle;

		PCt_0 = C_0 * P[0][0];
		PCt_1 = C_0 * P[1][0];

		E = R_angle + C_0 * PCt_0;

		K_0 = PCt_0 / E;
		K_1 = PCt_1 / E;

		t_0 = PCt_0;
		t_1 = C_0 * P[0][1];

		P[0][0] -= K_0 * t_0;
		P[0][1] -= K_0 * t_1;
		P[1][0] -= K_1 * t_0;
		P[1][1] -= K_1 * t_1;

		Angle += K_0 * angle_err;
		q_bias += K_1 * angle_err;
		Gyroscope = anglegyroscope - q_bias;
	}
}
