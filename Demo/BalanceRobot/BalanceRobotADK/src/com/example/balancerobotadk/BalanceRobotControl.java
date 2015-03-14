package com.example.balancerobotadk;

import android.content.Context;
import android.util.Log;
import android.view.Gravity;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup.LayoutParams;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.RelativeLayout;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;

import com.example.balancerobotadk.MainActivity;



public class BalanceRobotControl {
	
	public static void showAlgoPIDComponentDlg(Context context, View parent) {
		final String TAG = "BalanceRobotControl";
		final TextView textView_KAngle;
		final TextView textView_KAngleSpeed;
		final TextView textView_KPosition;
		final TextView textView_KPositionSpeed;
		final TextView textView_KBaseAngle;
		final SeekBar seekBar_KAngle;
		final SeekBar seekBar_KAngleSpeed;
		final SeekBar seekBar_KPosition;
		final SeekBar seekBar_KPositionSpeed;
		final SeekBar seekBar_KBaseAngle;
		View paramView = View.inflate(context, R.layout.param_dlg, null);
		LinearLayout contentView = (LinearLayout) paramView
				.findViewById(R.id.param_dlg_content_view);
		View algoLineParamView = View
				.inflate(context, R.layout.algo_pid, null);
		//final EditText algoLineEdit = (EditText) algoLineParamView
				//.findViewById(R.id.algo_line_edit);
		//algoLineEdit.setText(component.getKAngle() + "");
		contentView.addView(algoLineParamView);
		Button okBtn = (Button) paramView.findViewById(R.id.param_dlg_ok);
		Button canelBtn = (Button) paramView.findViewById(R.id.param_dlg_canel);
		final PopupWindow pinParamDialog = new PopupWindow(paramView,
				LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT, true);
		
		textView_KAngle = (TextView) paramView.findViewById(R.id.textView_KAngle);
		textView_KAngleSpeed = (TextView) paramView.findViewById(R.id.textView_KAngleSpeed);
		textView_KPosition = (TextView) paramView.findViewById(R.id.textView_KPosition);
		textView_KPositionSpeed = (TextView) paramView.findViewById(R.id.textView_KPositionSpeed);
		textView_KBaseAngle = (TextView) paramView.findViewById(R.id.textView_KBaseAngle);
		
		seekBar_KAngle = (SeekBar) paramView.findViewById(R.id.seekBar_KAngle);
		seekBar_KAngle.setMax(100);
		seekBar_KAngle.setProgress(MainActivity.getKAngle());
		
		seekBar_KAngleSpeed = (SeekBar) paramView.findViewById(R.id.seekBar_KAngleSpeed);
		seekBar_KAngleSpeed.setMax(100);
		seekBar_KAngleSpeed.setProgress(MainActivity.getKAngleSpeed());
		
		seekBar_KBaseAngle = (SeekBar) paramView.findViewById(R.id.seekBar_KBaseAngle);
		seekBar_KBaseAngle.setMax(100);
		seekBar_KBaseAngle.setProgress(MainActivity.getKBaseAngle());
		
		seekBar_KPosition = (SeekBar) paramView.findViewById(R.id.seekBar_KPosition);
		seekBar_KPosition.setMax(100);
		seekBar_KPosition.setProgress(MainActivity.getKPosition());
		
		seekBar_KPositionSpeed = (SeekBar) paramView.findViewById(R.id.seekBar_KPositionSpeed);
		seekBar_KPositionSpeed.setMax(100);
		seekBar_KPositionSpeed.setProgress(MainActivity.getKPositionSpeed());
		
		OnSeekBarChangeListener seekBar_KAngle_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
					boolean fromUser) {
				//App.setKAngle(seekBar_KAngle.getProgress());
				MainActivity.setKAngle(seekBar_KAngle.getProgress());
				textView_KAngle.setText("KAngle=" + seekBar_KAngle.getProgress());
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

		};
		seekBar_KAngle.setOnSeekBarChangeListener(seekBar_KAngle_Listener);	
		
		OnSeekBarChangeListener seekBar_KAngleSpeed_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
					boolean fromUser) {
				MainActivity.setKAngleSpeed(seekBar_KAngleSpeed.getProgress());
				textView_KAngleSpeed.setText("KAngleSpeed=" + seekBar_KAngleSpeed.getProgress());
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

		};
		seekBar_KAngleSpeed.setOnSeekBarChangeListener(seekBar_KAngleSpeed_Listener);	
		
		OnSeekBarChangeListener seekBar_KPosition_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
					boolean fromUser) {
				MainActivity.setKPosition(seekBar_KPosition.getProgress());
				textView_KPosition.setText("KPosition=" + seekBar_KPosition.getProgress());
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

		};
		seekBar_KPosition.setOnSeekBarChangeListener(seekBar_KPosition_Listener);	
		
		OnSeekBarChangeListener seekBar_KPositionSpeed_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
					boolean fromUser) {
				MainActivity.setKPositionSpeed(seekBar_KPositionSpeed.getProgress());
				textView_KPositionSpeed.setText("KPositionSpeed=" + seekBar_KPositionSpeed.getProgress());
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

		};
		seekBar_KPositionSpeed.setOnSeekBarChangeListener(seekBar_KPositionSpeed_Listener);	
		
		OnSeekBarChangeListener seekBar_KBaseAngle_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
					boolean fromUser) {
				MainActivity.setKBaseAngle(seekBar_KBaseAngle.getProgress());
				textView_KBaseAngle.setText("KBaseAngle=" + seekBar_KBaseAngle.getProgress());
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

		};
		seekBar_KBaseAngle.setOnSeekBarChangeListener(seekBar_KBaseAngle_Listener);	

		okBtn.setOnClickListener(new OnClickListener() {

			@Override
			public void onClick(View v) {
				pinParamDialog.dismiss();
				//component.setScale(Float.parseFloat(algoLineEdit.getText()
						//.toString()));
			}
		});
		
		canelBtn.setOnClickListener(new OnClickListener() {
			
			@Override
			public void onClick(View v) {
				pinParamDialog.dismiss();
			}
		});
		pinParamDialog.showAtLocation(parent, Gravity.CENTER, 0, 0);
	}
	
	public static float Angle_Acceleration_X(float y,float z)		
	{
		float g = (float)Math.sqrt(z * z + y*y );
		float cos = y / g;
		float AccelerometerAngle = (float) Math.acos(cos) * 57.3f;
		return AccelerometerAngle;
	}
	
	public static float Angle_Acceleration_ThreeD_X(float y,float z)		
	{
		float g = (float)Math.sqrt(z * z + y*y );
		float sin = y / g;
		float AccelerometerAngle = (float) Math.asin(sin) * 57.3f;
		return AccelerometerAngle;
	}
	
	public static float Angle_Acceleration_Y(float x,float z)		
	{
		float g = (float)Math.sqrt(z * z + x*x );
		float sin = x / g;
		float AccelerometerAngle = (float) Math.asin(sin) * 57.3f;
		return AccelerometerAngle;
	}
	
	public static float Angle_Acceleration_Z(float x,float y)		
	{
		float g = (float)Math.sqrt(x * x + y*y );
		float sin = x / g;
		float AccelerometerAngle = (float) Math.asin(sin) * 57.3f;
		return AccelerometerAngle;
	}

	
	
}
