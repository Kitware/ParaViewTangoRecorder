/*
 * Copyright 2014 Google Inc. All Rights Reserved.
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

package com.projecttango.experiments.javapointcloud;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.Tango.OnTangoUpdateListener;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.projecttango.tangoutils.ModelMatCalculator;

import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager.NameNotFoundException;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.util.Random;

/**
 * Main Activity class for the Point Cloud Sample. Handles the connection to the
 * {@link Tango} service and propagation of Tango XyzIj data to OpenGL and
 * Layout views. OpenGL rendering logic is delegated to the {@link PCrenderer}
 * class.
 */
public class PointCloudActivity extends Activity implements OnClickListener {

    private static final String TAG = PointCloudActivity.class.getSimpleName();
    private static final int SECS_TO_MILLISECS = 1000;
    private Tango mTango;
    private TangoConfig mConfig;

    private PCRenderer mRenderer;
    private GLSurfaceView mGLView;

    private TextView mDeltaTextView;
    private TextView mPoseCountTextView;
    private TextView mPoseTextView;
    private TextView mQuatTextView;
    private TextView mPoseStatusTextView;
    private TextView mTangoEventTextView;
    private TextView mPointCountTextView;
    private TextView mTangoServiceVersionTextView;
    private TextView mApplicationVersionTextView;
    private TextView mAverageZTextView;
    private TextView mFrequencyTextView;

    private Button mFirstPersonButton;
    private Button mThirdPersonButton;
    private Button mTopDownButton;

    private int count;
    private int mPreviousPoseStatus;
    private float mDeltaTime;
    private float mPosePreviousTimeStamp;
    private float mXyIjPreviousTimeStamp;
    private float mCurrentTimeStamp;
    private String mServiceVersion;
    private boolean mIsTangoServiceConnected;

    // My variables
    private Button mTakeSnapButton;
    private TextView mFilesWrittenToSDCardTextView;
    private Switch mAutoModeSwitch;
    private Switch mRecordSwitch;

    private String mFilename;
    private int mNumberOfFilesWritten;
    private Boolean mTimeToTakeSnap;
    private Boolean mAutoMode;
    private int myRandomNumber;
    private Random mRandGenerator;
    private ArrayList<float[]> mPosePositionBuffer;
    private ArrayList<float[]> mPoseOrientationBuffer;
    private int mNumPoseInSequence;
    boolean mIsRecording;
    // End of My variables

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_jpoint_cloud);
        setTitle(R.string.app_name);

        mPoseTextView = (TextView) findViewById(R.id.pose);
        mQuatTextView = (TextView) findViewById(R.id.quat);
        mPoseCountTextView = (TextView) findViewById(R.id.posecount);
        mDeltaTextView = (TextView) findViewById(R.id.deltatime);
        mTangoEventTextView = (TextView) findViewById(R.id.tangoevent);
        mPoseStatusTextView = (TextView) findViewById(R.id.status);
        mPointCountTextView = (TextView) findViewById(R.id.pointCount);
        mTangoServiceVersionTextView = (TextView) findViewById(R.id.version);
        mApplicationVersionTextView = (TextView) findViewById(R.id.appversion);
        mAverageZTextView = (TextView) findViewById(R.id.averageZ);
        mFrequencyTextView = (TextView) findViewById(R.id.frameDelta);

        mFirstPersonButton = (Button) findViewById(R.id.first_person_button);
        mFirstPersonButton.setOnClickListener(this);
        mThirdPersonButton = (Button) findViewById(R.id.third_person_button);
        mThirdPersonButton.setOnClickListener(this);
        mTopDownButton = (Button) findViewById(R.id.top_down_button);
        mTopDownButton.setOnClickListener(this);

        mTango = new Tango(this);
        mConfig = mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);

        int maxDepthPoints = mConfig.getInt("max_point_cloud_elements");
        mRenderer = new PCRenderer(maxDepthPoints);
        mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);
        mGLView.setEGLContextClientVersion(2);
        mGLView.setRenderer(mRenderer);
        mGLView.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);

        PackageInfo packageInfo;
        try {
            packageInfo = this.getPackageManager().getPackageInfo(
                    this.getPackageName(), 0);
            mApplicationVersionTextView.setText(packageInfo.versionName);
        } catch (NameNotFoundException e) {
            e.printStackTrace();
        }

        // Display the version of Tango Service
        mServiceVersion = mConfig.getString("tango_service_library_version");
        mTangoServiceVersionTextView.setText(mServiceVersion);
        mIsTangoServiceConnected = false;

        // My initializations
        mTakeSnapButton = (Button) findViewById(R.id.take_snap_button);
        mTakeSnapButton.setOnClickListener(this);
        mFilesWrittenToSDCardTextView = (TextView) findViewById(R.id.fileWritten);
        mAutoModeSwitch = (Switch) findViewById(R.id.auto_mode_switch);
        mAutoModeSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                autoMode_SwitchChanged(isChecked);
            }
        });
        mRecordSwitch = (Switch) findViewById(R.id.record_switch);
        mRecordSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                record_SwitchChanged(isChecked);
            }
        });

        mFilename = "";
        mNumberOfFilesWritten = 0;
        mTimeToTakeSnap = false;
        mAutoMode = false;
        mAutoModeSwitch.setChecked(false);
        mIsRecording = false;
        mRecordSwitch.setChecked(false);
        mRandGenerator = new Random();
        mPosePositionBuffer = new ArrayList<float[]>();
        mPoseOrientationBuffer = new ArrayList<float[]>();
        mNumPoseInSequence = 0;
        // End of My initializations
    }

    @Override
    protected void onPause() {
        super.onPause();
        try {
            mTango.disconnect();
            mIsTangoServiceConnected = false;
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError,
                    Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (!mIsTangoServiceConnected) {
            startActivityForResult(
                    Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_MOTION_TRACKING),
                    Tango.TANGO_INTENT_ACTIVITYCODE);
        }
        Log.i(TAG, "onResumed");
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // Check which request we're responding to
        if (requestCode == Tango.TANGO_INTENT_ACTIVITYCODE) {
            Log.i(TAG, "Triggered");
            // Make sure the request was successful
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(this, R.string.motiontrackingpermission,
                        Toast.LENGTH_LONG).show();
                finish();
                return;
            }
            try {
                setTangoListeners();
            } catch (TangoErrorException e) {
                Toast.makeText(this, R.string.TangoError, Toast.LENGTH_SHORT)
                        .show();
            } catch (SecurityException e) {
                Toast.makeText(getApplicationContext(),
                        R.string.motiontrackingpermission, Toast.LENGTH_SHORT)
                        .show();
            }
            try {
                mTango.connect(mConfig);
                mIsTangoServiceConnected = true;
            } catch (TangoOutOfDateException e) {
                Toast.makeText(getApplicationContext(),
                        R.string.TangoOutOfDateException, Toast.LENGTH_SHORT)
                        .show();
            } catch (TangoErrorException e) {
                Toast.makeText(getApplicationContext(), R.string.TangoError,
                        Toast.LENGTH_SHORT).show();
            }
            setUpExtrinsics();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.first_person_button:
                mRenderer.setFirstPersonView();
                break;
            case R.id.third_person_button:
                mRenderer.setThirdPersonView();
                break;
            case R.id.top_down_button:
                mRenderer.setTopDownView();
                break;
            case R.id.take_snap_button:
                takeSnapshot_ButtonClicked();
                break;
            default:
                Log.w(TAG, "Unrecognized button click.");
                break;
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        return mRenderer.onTouchEvent(event);
    }

    private void setUpExtrinsics() {
        // Set device to imu matrix in Model Matrix Calculator.
        TangoPoseData device2IMUPose = new TangoPoseData();
        TangoCoordinateFramePair framePair = new TangoCoordinateFramePair();
        framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_DEVICE;
        try {
            device2IMUPose = mTango.getPoseAtTime(0.0, framePair);
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError,
                    Toast.LENGTH_SHORT).show();
        }
        mRenderer.getModelMatCalculator().SetDevice2IMUMatrix(
                device2IMUPose.getTranslationAsFloats(),
                device2IMUPose.getRotationAsFloats());

        // Set color camera to imu matrix in Model Matrix Calculator.
        TangoPoseData color2IMUPose = new TangoPoseData();

        framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR;
        try {
            color2IMUPose = mTango.getPoseAtTime(0.0, framePair);
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError,
                    Toast.LENGTH_SHORT).show();
        }
        mRenderer.getModelMatCalculator().SetColorCamera2IMUMatrix(
                color2IMUPose.getTranslationAsFloats(),
                color2IMUPose.getRotationAsFloats());
    }

    private void setTangoListeners() {
        // Configure the Tango coordinate frame pair
        final ArrayList<TangoCoordinateFramePair> framePairs =
                new ArrayList<TangoCoordinateFramePair>();
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));
        // Listen for new Tango data
        mTango.connectListener(framePairs, new OnTangoUpdateListener() {

            @Override
            public void onPoseAvailable(final TangoPoseData pose) {
                mDeltaTime = (float) (pose.timestamp - mPosePreviousTimeStamp)
                        * SECS_TO_MILLISECS;
                mPosePreviousTimeStamp = (float) pose.timestamp;
                if (mPreviousPoseStatus != pose.statusCode) {
                    count = 0;
                }
                count++;
                mPreviousPoseStatus = pose.statusCode;

                // My pose buffering
                if (mIsRecording && pose.statusCode == TangoPoseData.POSE_VALID) {
                    mPosePositionBuffer.add(mNumPoseInSequence, pose.getTranslationAsFloats());
                    mPoseOrientationBuffer.add(mNumPoseInSequence, pose.getRotationAsFloats());
                    mNumPoseInSequence++;
                }
                //End of My pose buffering

                mRenderer.getModelMatCalculator().updateModelMatrix(
                        pose.getTranslationAsFloats(),
                        pose.getRotationAsFloats());
                mRenderer.updateViewMatrix();
                mGLView.requestRender();
                // Update the UI with TangoPose information
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        DecimalFormat threeDec = new DecimalFormat("0.000");
                        String translationString = "["
                                + threeDec.format(pose.translation[0]) + ", "
                                + threeDec.format(pose.translation[1]) + ", "
                                + threeDec.format(pose.translation[2]) + "] ";
                        String quaternionString = "["
                                + threeDec.format(pose.rotation[0]) + ", "
                                + threeDec.format(pose.rotation[1]) + ", "
                                + threeDec.format(pose.rotation[2]) + ", "
                                + threeDec.format(pose.rotation[3]) + "] ";

                        // Display pose data on screen in TextViews
                        mPoseTextView.setText(translationString);
                        mQuatTextView.setText(quaternionString);
                        mPoseCountTextView.setText(Integer.toString(count));
                        mDeltaTextView.setText(threeDec.format(mDeltaTime));
                        if (pose.statusCode == TangoPoseData.POSE_VALID) {
                            mPoseStatusTextView.setText(R.string.pose_valid);
                        } else if (pose.statusCode == TangoPoseData.POSE_INVALID) {
                            mPoseStatusTextView.setText(R.string.pose_invalid);
                        } else if (pose.statusCode == TangoPoseData.POSE_INITIALIZING) {
                            mPoseStatusTextView.setText(R.string.pose_initializing);
                        } else if (pose.statusCode == TangoPoseData.POSE_UNKNOWN) {
                            mPoseStatusTextView.setText(R.string.pose_unknown);
                        }
                    }
                });
            }

            @Override
            public void onXyzIjAvailable(final TangoXyzIjData xyzIj) {
                mCurrentTimeStamp = (float) xyzIj.timestamp;
                final float frameDelta = (mCurrentTimeStamp - mXyIjPreviousTimeStamp)
                        * SECS_TO_MILLISECS;
                mXyIjPreviousTimeStamp = mCurrentTimeStamp;
                byte[] buffer = new byte[xyzIj.xyzCount * 3 * 4];
                FileInputStream fileStream = new FileInputStream(
                        xyzIj.xyzParcelFileDescriptor.getFileDescriptor());
                try {
                    fileStream.read(buffer, xyzIj.xyzParcelFileDescriptorOffset, buffer.length);
                    fileStream.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }

                // My writing to file function
                writePointCloudToFile(xyzIj, buffer, framePairs);
                // End of My writing to file function

                try {
                    TangoPoseData pointCloudPose = mTango.getPoseAtTime(
                            mCurrentTimeStamp, framePairs.get(0));

                    mRenderer.getPointCloud().UpdatePoints(buffer,
                            xyzIj.xyzCount);
                    mRenderer.getModelMatCalculator()
                            .updatePointCloudModelMatrix(
                                    pointCloudPose.getTranslationAsFloats(),
                                    pointCloudPose.getRotationAsFloats());
                    mRenderer.getPointCloud().setModelMatrix(
                            mRenderer.getModelMatCalculator()
                                    .getPointCloudModelMatrixCopy());
                } catch (TangoErrorException e) {
                    Toast.makeText(getApplicationContext(),
                            R.string.TangoError, Toast.LENGTH_SHORT).show();
                } catch (TangoInvalidException e) {
                    Toast.makeText(getApplicationContext(),
                            R.string.TangoError, Toast.LENGTH_SHORT).show();
                }

                // Must run UI changes on the UI thread. Running in the Tango
                // service thread
                // will result in an error.
                runOnUiThread(new Runnable() {
                    DecimalFormat threeDec = new DecimalFormat("0.000");

                    @Override
                    public void run() {
                        // Display number of points in the point cloud
                        mPointCountTextView.setText(Integer
                                .toString(xyzIj.xyzCount));
                        mFrequencyTextView.setText(""
                                + threeDec.format(frameDelta));
                        mAverageZTextView.setText(""
                                + threeDec.format(mRenderer.getPointCloud()
                                .getAverageZ()));
                        // My GUI updates
                        mFilesWrittenToSDCardTextView.setText("" +
                                String.valueOf(mNumberOfFilesWritten) + "\n" + mFilename);
                        // End of My GUI updates
                    }
                });
            }

            @Override
            public void onTangoEvent(final TangoEvent event) {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        mTangoEventTextView.setText(event.eventKey + ": "
                                + event.eventValue);
                    }
                });
            }
        });
    }


    // My functions

    // This function is called when the Take Snapshot button is clicked
    private void takeSnapshot_ButtonClicked() {
        if(!mIsRecording) {
            myRandomNumber = mRandGenerator.nextInt(0xFFFFFF);
            mNumberOfFilesWritten = 0;
        }
        mTimeToTakeSnap=true;
    }

    // This function is called when the Auto Mode Switch is changed
    private void autoMode_SwitchChanged(boolean isChecked) {
        mAutoMode = isChecked;
    }

    // This function is called when the Record Switch is changed
    private void record_SwitchChanged(boolean isChecked) {
        mIsRecording = isChecked;
        // Start Recording
        if (mIsRecording) {
            // Generate a new random number to create a new group of files
            myRandomNumber = mRandGenerator.nextInt(0xFFFFFF);
            mNumberOfFilesWritten = 0;
        }
        // Finish Recording
        else {
            // Stop the Pose Recording, and write them to a file.
            writePoseToFile(mNumPoseInSequence);
            mNumPoseInSequence = 0;
            mPoseOrientationBuffer.clear(); // TODO: Might not be necessary, remove for performance
            mPoseOrientationBuffer.clear(); // TODO: Might not be necessary, remove for performance
        }
    }

    // This function writes the XYZ points to .vtk files
    private void writePointCloudToFile(TangoXyzIjData xyzIj, byte[] buffer,
                                       ArrayList<TangoCoordinateFramePair> framePairs) {

        // Saving the frame or not, depending on the current mode.
        if(!mAutoMode && mTimeToTakeSnap  || mAutoMode && count%10 == 0) {

            ByteBuffer myBuffer = ByteBuffer.allocate(xyzIj.xyzCount * 3 * 4);
            myBuffer.order(ByteOrder.LITTLE_ENDIAN);
            myBuffer.put(buffer, xyzIj.xyzParcelFileDescriptorOffset, myBuffer.capacity());

//            Calendar rightNow = Calendar.getInstance();
//            int month = rightNow.get(Calendar.MONTH);
//            int day = rightNow.get(Calendar.DAY_OF_MONTH);
//            int hour = rightNow.get(Calendar.HOUR_OF_DAY);
//            int minute = rightNow.get(Calendar.MINUTE);
//            int sec = rightNow.get(Calendar.SECOND);
//            int milliSec = rightNow.get(Calendar.MILLISECOND);

            File sdCard = Environment.getExternalStorageDirectory();
            File dir = new File(sdCard.getAbsolutePath() + "/Tango/MyPointCloudData");
            mFilename = "pc-" +  Integer.toHexString(myRandomNumber) + "-" +
                    String.format("%03d", mNumberOfFilesWritten+1)  + ".vtk";

            File file = new File(dir, mFilename);


            //TODO : Write data in binary to improve writing speed
            try {
                // get external storage file reference
                FileWriter writer = new FileWriter(file);
                // Writes the content to the file
                writer.write("# vtk DataFile Version 3.0\n" +
                        "vtk output\n" +
                        "ASCII\n" +
                        "DATASET POLYDATA\n" +
                        "POINTS " + xyzIj.xyzCount + " float\n");

                for (int i = 0; i < xyzIj.xyzCount; i++) {

                    writer.write(String.valueOf(myBuffer.getFloat(3 * i * 4)) + " " +
                            String.valueOf(myBuffer.getFloat((3 * i + 1) * 4)) + " " +
                            String.valueOf(myBuffer.getFloat((3 * i + 2) * 4)) + " ");
                    if((i+1)%3 ==0) {
                        writer.write("\n");
                    }
                }

                writer.write("\n\nVERTICES 1 " + String.valueOf(xyzIj.xyzCount+1) + "\n" +
                        xyzIj.xyzCount);
                for (int i = 0; i < xyzIj.xyzCount; i++) {
                    writer.write(" "+i);
                }
                writer.close();
                mNumberOfFilesWritten++;
                mTimeToTakeSnap = false;
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    }



    private void writePoseToFile(int numPoints) {

        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File(sdCard.getAbsolutePath() + "/Tango/MyPointCloudData");
        String poseFileName = "pc-" +  Integer.toHexString(myRandomNumber) + "-poses.vtk";
        File file = new File(dir, poseFileName);

        //TODO : Write data in binary to improve writing speed
        try {
            // get external storage file reference
            FileWriter writer = new FileWriter(file);
            // Writes the content to the file
            writer.write("# vtk DataFile Version 3.0\n" +
                    "vtk output\n" +
                    "ASCII\n" +
                    "DATASET POLYDATA\n" +
                    "POINTS " + numPoints + " float\n");

            for (int i = 0; i < numPoints; i++) {

                writer.write(String.valueOf(mPosePositionBuffer.get(i)[0]) + " " +
                        String.valueOf(mPosePositionBuffer.get(i)[1]) + " " +
                        String.valueOf(mPosePositionBuffer.get(i)[2]) + " ");
                if((i+1)%3 ==0) {
                    writer.write("\n");
                }
            }

            writer.write("\n\nLINES 1 " + String.valueOf(numPoints+1) + "\n" +
                    numPoints);
            for (int i = 0; i < numPoints; i++) {
                writer.write(" "+i);
            }

            writer.write("\n\nPOINT_DATA " + String.valueOf(numPoints) + "\n" +
                    "FIELD FieldData 1\n" +
                    "orientation 4 " + String.valueOf(numPoints) + " float\n" );

            for (int i = 0; i < numPoints; i++) {

                writer.write(String.valueOf(mPoseOrientationBuffer.get(i)[0]) + " " +
                        String.valueOf(mPoseOrientationBuffer.get(i)[1]) + " " +
                        String.valueOf(mPoseOrientationBuffer.get(i)[2]) + " " +
                        String.valueOf(mPoseOrientationBuffer.get(i)[3]));
                if((i+1)%3 ==0) {
                    writer.write("\n");
                }
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // End of My functions
}
