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
 *
 */

/*
 * Additional changes are made by Kitware Inc. and are also licensed under
 * the Apache License, Version 2.0 (the "License"):
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 */

package com.kitware.tangoproject.paraviewtangorecorder;

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

import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager.NameNotFoundException;
import android.net.Uri;
import android.opengl.GLSurfaceView;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.support.v4.content.FileProvider;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.ProgressBar;
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
import java.util.concurrent.Semaphore;

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

    private int mValidPoseCallbackCount;
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
    private ProgressBar mWaitForIt;

    private static final String mSaveDirAbsPath = Environment.getExternalStorageDirectory()
            .getAbsolutePath() + "/Tango/MyPointCloudData/";
    private String mFilename;
    private int mNumberOfFilesWritten;
    private Boolean mTimeToTakeSnap;
    private Boolean mAutoMode;
    private double myDateNumber;
    private ArrayList<float[]> mPosePositionBuffer;
    private ArrayList<float[]> mPoseOrientationBuffer;
    private ArrayList<Float> mPoseTimestampBuffer;
    private ArrayList<String> mFilenameBuffer;
    private int mNumPoseInSequence;
    boolean mIsRecording;
    private int mXyzIjCallbackCount;
    private Semaphore mutex_on_mIsRecording;
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
        mWaitForIt = (ProgressBar) findViewById(R.id.progressBar);
        mWaitForIt.setVisibility(View.VISIBLE);


        mFilename = "";
        mNumberOfFilesWritten = 0;
        mTimeToTakeSnap = false;
        mTakeSnapButton.setEnabled(false);
        mAutoMode = false;
        mAutoModeSwitch.setChecked(false);
        mIsRecording = false;
        mRecordSwitch.setChecked(false);
        mPosePositionBuffer = new ArrayList<float[]>();
        mPoseOrientationBuffer = new ArrayList<float[]>();
        mPoseTimestampBuffer = new ArrayList<Float>();
        mFilenameBuffer = new ArrayList<String>();
        mNumPoseInSequence = 0;
        mXyzIjCallbackCount = 0;
        mutex_on_mIsRecording = new Semaphore(1,true);
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
                    mValidPoseCallbackCount = 0;
                }
                mValidPoseCallbackCount++;
                mPreviousPoseStatus = pose.statusCode;

                // My pose buffering
                if (mIsRecording && pose.statusCode == TangoPoseData.POSE_VALID) {
                    mPosePositionBuffer.add(mNumPoseInSequence, pose.getTranslationAsFloats());
                    mPoseOrientationBuffer.add(mNumPoseInSequence, pose.getRotationAsFloats());
                    mPoseTimestampBuffer.add((float)pose.timestamp);
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
                        mPoseCountTextView.setText(Integer.toString(mValidPoseCallbackCount));
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
                mXyzIjCallbackCount++;
                final byte[] buffer = new byte[xyzIj.xyzCount * 3 * 4];
                FileInputStream fileStream = new FileInputStream(
                        xyzIj.xyzParcelFileDescriptor.getFileDescriptor());
                try {
                    fileStream.read(buffer, xyzIj.xyzParcelFileDescriptorOffset, buffer.length);
                    fileStream.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }

                // My writing to file function

                // Background task for writing to file
                class SendCommandTask extends AsyncTask<Void, Void, Boolean> {
                    /** The system calls this to perform work in a worker thread and
                     * delivers it the parameters given to AsyncTask.execute() */
                    @Override
                    protected Boolean doInBackground(Void... params) {

                        try {
                            mutex_on_mIsRecording.acquire();
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        // Saving the frame or not, depending on the current mode.
                        if ( mTimeToTakeSnap || ( mIsRecording && mAutoMode && mXyzIjCallbackCount % 3 == 0 ) ) {
                            writePointCloudToFile(xyzIj, buffer, framePairs);
                        }
                        mutex_on_mIsRecording.release();
                        return true;
                    }

                    /** The system calls this to perform work in the UI thread and delivers
                     * the result from doInBackground() */
                    @Override
                    protected void onPostExecute(Boolean done) {

                    }
                }
                new SendCommandTask().execute();



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
                        mWaitForIt.setVisibility(View.GONE);
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
        mTimeToTakeSnap=true;
    }

    // This function is called when the Auto Mode Switch is changed
    private void autoMode_SwitchChanged(boolean isChecked) {
        mAutoMode = isChecked;
    }

    // This function is called when the Record Switch is changed
    private void record_SwitchChanged(boolean isChecked) {
        try {
            mutex_on_mIsRecording.acquire();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        mIsRecording = isChecked;
        // Start Recording
        if (mIsRecording) {
            // Generate a new date number to create a new group of files
            Calendar rightNow = Calendar.getInstance();
            int hour = rightNow.get(Calendar.HOUR_OF_DAY);
            int minute = rightNow.get(Calendar.MINUTE);
            int sec = rightNow.get(Calendar.SECOND);
            int milliSec = rightNow.get(Calendar.MILLISECOND);
            myDateNumber = 10000*hour + 100*minute + sec + milliSec/1000.0;
            mNumberOfFilesWritten = 0;
            mTakeSnapButton.setEnabled(true);
        }
        // Finish Recording
        else {
            mTakeSnapButton.setEnabled(false);
            // Stop the Pose Recording, and write them to a file.
            writePoseToFile(mNumPoseInSequence);
            // If a snap has been asked just before, but not saved, ignore it, otherwise,
            // it will be saved at the end dof this function, and the 2nd archive will override
            // the first.
            mTimeToTakeSnap = false;
            mNumPoseInSequence = 0;
            mPoseOrientationBuffer.clear();
            mPoseOrientationBuffer.clear();
            mPoseTimestampBuffer.clear();

            // Zip all the files from this sequence
            String zipFilename = mSaveDirAbsPath + "TangoData_" +
                    (int)myDateNumber + "" + (int)((myDateNumber%1)*100) +
                    "_" + mFilenameBuffer.size() + "files.zip";
            String[] fileList = mFilenameBuffer.toArray(new String[mFilenameBuffer.size()]);
            ZipWriter zipper = new ZipWriter(fileList, zipFilename);
            zipper.zip();

            // Delete the data files now that they are archived
            for (String s : mFilenameBuffer) {
                File file = new File(s);
                boolean deleted = file.delete();
                if (!deleted) {
                    Log.w(TAG, "File \"" + s + "\" not deleted\n");
                }
            }
            mFilenameBuffer.clear();

            // Send the zip file to another app
            File myZipFile = new File(zipFilename);
            Uri myZipUri = FileProvider.getUriForFile(this, "com.kitware.tangoproject." +
                    "paraviewtangorecorder.fileprovider", myZipFile);
            Intent shareIntent = new Intent();
            shareIntent.setAction(Intent.ACTION_SEND);
            shareIntent.putExtra(Intent.EXTRA_STREAM, myZipUri);
            shareIntent.setType("application/zip");
            startActivity(Intent.createChooser(shareIntent, "Send Data To..."));
        }
        mutex_on_mIsRecording.release();

    }

    // This function writes the XYZ points to .vtk files
    private void writePointCloudToFile(TangoXyzIjData xyzIj, byte[] buffer,
                                       ArrayList<TangoCoordinateFramePair> framePairs) {

        ByteBuffer myBuffer = ByteBuffer.allocate(xyzIj.xyzCount * 3 * 4);
        myBuffer.order(ByteOrder.LITTLE_ENDIAN);
        myBuffer.put(buffer, xyzIj.xyzParcelFileDescriptorOffset, myBuffer.capacity());

        File dir = new File(mSaveDirAbsPath);
        if(!dir.exists()) {
            boolean created = dir.mkdir();
            if (created) {
                Log.i(TAG, "Folder: \"" + mSaveDirAbsPath + "\" created\n");
            }
        }
        String nowTime = (int)myDateNumber + "" + (int)((myDateNumber%1)*100);
        mFilename = "pc_" + nowTime + "_" + String.format("%03d", mNumberOfFilesWritten) + ".vtk";
        mFilenameBuffer.add(mSaveDirAbsPath + mFilename);
        File file = new File(dir, mFilename);


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
                if ((i + 1) % 3 == 0) {
                    writer.write("\n");
                }
            }

            writer.write("\n\nVERTICES 1 " + String.valueOf(xyzIj.xyzCount + 1) + "\n" +
                    xyzIj.xyzCount);
            for (int i = 0; i < xyzIj.xyzCount; i++) {
                writer.write(" " + i);
            }

            writer.write("\n\nFIELD FieldData 1\n" +
                    "timestamp 1 1 float\n" );
            writer.write(String.valueOf((float)xyzIj.timestamp));


            writer.close();
            mNumberOfFilesWritten++;
            mTimeToTakeSnap = false;

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void writePoseToFile(int numPoints) {

        File dir = new File(mSaveDirAbsPath);
        if(!dir.exists()) {
            boolean created = dir.mkdir();
            if (created) {
                Log.i(TAG, "Folder: \"" + mSaveDirAbsPath + "\" created\n");
            }
        }
        String poseFileName = "pc_" +  (int)myDateNumber + "" + (int)((myDateNumber%1)*100) +
                "_poses.vtk";
        mFilenameBuffer.add(mSaveDirAbsPath + poseFileName);
        File file = new File(dir, poseFileName);

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
                    "FIELD FieldData 2\n" +
                    "orientation 4 " + String.valueOf(numPoints) + " float\n" );

            for (int i = 0; i < numPoints; i++) {

                writer.write(String.valueOf(mPoseOrientationBuffer.get(i)[0]) + " " +
                        String.valueOf(mPoseOrientationBuffer.get(i)[1]) + " " +
                        String.valueOf(mPoseOrientationBuffer.get(i)[2]) + " " +
                        String.valueOf(mPoseOrientationBuffer.get(i)[3]) + " ");
                if((i+1)%3 ==0) {
                    writer.write("\n");
                }
            }

            writer.write("\n\ntimestamp 1 " + String.valueOf(numPoints) + " float\n" );
            for (int i = 0; i < numPoints; i++) {

                writer.write(String.valueOf(mPoseTimestampBuffer.get(i)) + " ");
                if((i+1)%9 ==0) {
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
