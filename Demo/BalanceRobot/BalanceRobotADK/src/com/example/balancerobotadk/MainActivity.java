package com.example.balancerobotadk;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.MediaPlayer;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Context;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;

import com.example.balancerobotadk.BalanceRobotControl;
import com.example.service.usb.USBAccessoryManager;
import com.example.service.usb.USBAccessoryManagerMessage;

public class MainActivity extends Activity {
	
	private final static int USBAccessoryWhat = 0;
	private SensorManager mSensorManager;
	private USBAccessoryManager accessoryManager;
	
	private Context mContext;
	private static MainActivity mMainActivity;
	private Button PIDAdjuster;
	private View menuView;
	
	private String TAG = "Hippo";
	
	public boolean modelOf405 = true;
	
	byte[] shootCommandPacket = new byte[13];
	byte[] controlCommandPacket = new byte[13];
	byte[] readcommandPacket = new byte[12];
	private float Sum_Measure_Gyroscope;
	private float Angle;
	private float Gyroscope;
	private float Angle_Acceleration;
	private int   speed_mr = 0;		 
	private int   speed_ml = 0;		 
	private float speed_r_l = 0;	
	private float speed = 0;        
	private float position = 0;	    
	private float SpeedNeed = 0;
	private float TurnNeed = 0;
	private float TurnNeed_US = 0;
	
	/*V0.1
	public static int KAngle = 56;
	public static int KAngleSpeed = 47;
	public static int KPosition = 80;
	public static int KPositionSpeed = 60;
	public static int KBaseAngle = 23;
	*/
	
	//V0.2
	public static int KAngle = 67;
	public static int KAngleSpeed = 64;
	public static int KPosition = 50;
	public static int KPositionSpeed = 78;
	public static int KBaseAngle = 11;
	
	
	@SuppressLint("HandlerLeak")
	private Handler handler = new Handler() {

		@Override
		public void handleMessage(Message msg) {
			// TODO Auto-generated method stub
			// super.handleMessage(msg);
			switch(msg.what)
			{	
				case USBAccessoryWhat:
					switch(((USBAccessoryManagerMessage)msg.obj).type) {
					case READ:	
						if(accessoryManager.available() < 4) {
							//All of our commands in this example are 4 bytes.  If there are less
							//  than 4 bytes left, it is a partial command
							break;
						}
						byte[] commandPacket405 = new byte[4];
						accessoryManager.read(commandPacket405);
						speed_mr = commandPacket405[1]&0xFF;
						if(commandPacket405[0] == 0x00){
							speed_mr *= -1;
						}
						speed_ml = commandPacket405[3]&0xFF;
						if(commandPacket405[2] == 0x00){
							speed_ml *= -1;
						}
						speed_r_l = (speed_mr + speed_ml)*0.5f;
						speed *= 0.7f;		                  //
						speed += speed_r_l*0.3f;
						position += speed;	                  //
						if(modelOf405){
						    if(position<-360) position = -360; 
							if(position> 360) position =  360;
						}else{
						    if(position<-360) position = -360; 
							if(position> 360) position =  360;
						}
						position += SpeedNeed;						
					}
				break;
			}
		}		
	};	
	
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		mContext = this;
		mMainActivity = this;
		LayoutInflater mLayoutInflater = (LayoutInflater) getSystemService(LAYOUT_INFLATER_SERVICE);
		menuView = (View) mLayoutInflater.inflate(
				R.layout.usbbalancerobotactivity, null, true);
		mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		accessoryManager = new USBAccessoryManager(handler, USBAccessoryWhat);
		PIDAdjuster = (Button) findViewById(R.id.PIDAdjuster);
		PIDAdjuster.setOnClickListener(PIDAdjusterClickListener);
		controlCommandPacket[4] = 0x08;
		shootCommandPacket[4] = 0x08;
	}
	
	private SensorEventListener mSensorEventListener = new SensorEventListener() {
		@Override
		public void onSensorChanged(SensorEvent event) {
			if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
				Angle_Acceleration =  BalanceRobotControl.Angle_Acceleration_X(
						event.values[SensorManager.DATA_Z],
						event.values[SensorManager.DATA_Y]) - 90;
				
			}else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
				float Measure_Gyroscope = 57.3f * (event.values[SensorManager.DATA_X]) + 1.5f;
				Sum_Measure_Gyroscope += Measure_Gyroscope * 0.01F;
				Kalman_Filter(Angle_Acceleration,Measure_Gyroscope);
				//float PWM = -1*(KAngle*(Angle - 7.0f + KBaseAngle*0.1f)*0.8f + KAngleSpeed*Gyroscope*0.05f + KPosition*position*0.01f + KPositionSpeed*speed*0.5f);//V0.1
				float PWM = -1*(KAngle*(Angle - 7.0f + KBaseAngle*0.1f)*0.8f + KAngleSpeed*Gyroscope*0.05f + KPosition*position*0.02f + KPositionSpeed*speed*0.5f);//V0.2
				int PWMLeft = (int)(PWM + TurnNeed + TurnNeed_US);
				int PWMRight = (int)(PWM - TurnNeed - TurnNeed_US);
				if(PWMLeft > 255){
					PWMLeft = 255;
				}
				if(PWMLeft < -255){
					PWMLeft = -255;
				}
				if(PWMRight > 255){
					PWMRight = 255;
				}
				if(PWMRight < -255){
					PWMRight = -255;
				}
				if(modelOf405){
					PWMLeft *= -1;
					PWMRight *= -1;
					for (int i = 8; i > 4; i--) {
						controlCommandPacket[i] = (byte) (PWMLeft >>> (24 - (8-i) * 8));
					}
					for (int i = 12; i > 8; i--) {
						controlCommandPacket[i] = (byte) (PWMRight >>> (24 - (12-i)  * 8));
					}	
				}else{
					for (int i = 5; i < 9; i++) {
						controlCommandPacket[i] = (byte) (PWMLeft >>> (24 - (i - 5) * 8));
					}
					for (int i = 9; i < 13; i++) {
						controlCommandPacket[i] = (byte) (PWMRight >>> (24 - (i - 9)  * 8));
					}	
				}
				controlCommandPacket[0] = 0x00;
				accessoryManager.write(controlCommandPacket);
			}
		}
		@Override
		public void onAccuracyChanged(Sensor sensor, int accuracy) {

		}
	};

	private OnClickListener PIDAdjusterClickListener = new OnClickListener() {
		@Override
		public void onClick(View arg0) {
			BalanceRobotControl.showAlgoPIDComponentDlg(mContext, menuView);
		}
	};
	
	public void onResume() {
		super.onResume();
		accessoryManager.enable(this, getIntent());
		//accessoryManager.enable(this);
		mSensorManager.registerListener(mSensorEventListener,
				mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
				SensorManager.SENSOR_DELAY_UI);
		mSensorManager.registerListener(mSensorEventListener,
				mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), 8000);
	}
/*
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}
*/	
	public static int getKAngle() {
		return KAngle;
	}
	
	public static void setKAngle(int kangle) {
		mMainActivity.KAngle = kangle;
	}
	
	public static int getKAngleSpeed() {
		return KAngleSpeed;
	}
	
	public static void setKAngleSpeed(int kangleSpeed) {
		mMainActivity.KAngleSpeed = kangleSpeed;
	}
	
	public static int getKPosition() {
		return KPosition;
	}

	public static void setKPosition(int kposition) {
		mMainActivity.KPosition = kposition;
	}
	
	public static int getKPositionSpeed() {
		return KPositionSpeed;
	}

	public static void setKPositionSpeed(int kpositionspeed) {
		mMainActivity.KPositionSpeed = kpositionspeed;
	}
	
	public static int getKBaseAngle() {
		return KBaseAngle;
	}

	public static void setKBaseAngle(int kbaseangle) {
		mMainActivity.KBaseAngle = kbaseangle;
	}
	
	private static float Q_angle=0.005f, Q_gyro=0.003f, R_angle=0.5f, dt=0.008f;
	private static float q_bias=0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
	private static float P[][] = { { 1, 0 }, { 0, 1 } };
	private static float Pdot[] ={0,0,0,0};
	private static char C_0 = 1;
	public void Kalman_Filter(float angleaccelerometer,float anglegyroscope)		
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
