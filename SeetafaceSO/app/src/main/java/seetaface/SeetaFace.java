package seetaface;

/*
 *  Created by hph  2017-06-23
 *  移植中科院山世光老师的开源人脸识别工程SeetaFace
 */

import android.graphics.Bitmap;
import android.os.Environment;

import com.csipsimple.utils.Log;
import com.soben.model.*;

public class SeetaFace {
    private final static String TAG = "SeetaFace";
    private static SeetaFace instance;
    // 训练模型目录
    private final static String FaceModelDir = Environment.getExternalStorageDirectory().getAbsolutePath();
    // 人脸检测模型文件
    private final static String SeetaFdPath = FaceModelDir + "/seeta_fd_frontal_v1.0.bin";
    // 人脸对齐模型文件
    private final static String SeetaFaPath = FaceModelDir + "/seeta_fa_v1.1.bin";
    // 人脸识别模型文件
    private final static String SeetaFrPath = FaceModelDir + "/seeta_fr_v1.0.bin";

    private boolean initSeetaFaceSuccess = false;
    private boolean isDoingInit = false;
    private boolean _isInitFaild = false;

    static {
        System.loadLibrary("SeetafaceSo");
    }

    public static SeetaFace getInstance() {
        if(instance == null){
            try {
                instance = new SeetaFace();
            }
            catch (Exception e){
                e.printStackTrace();
            }
        }

        if(!instance.initSeetaFaceSuccess && !instance.isDoingInit){
            new Thread(new Runnable() {
                @Override
                public void run() {
                    instance.isDoingInit = true;
                    try {
                        if(checkModelFilesIsExists()) {
                            Log.d(TAG, "start init ");
                            instance.initSeetaFaceSuccess = instance.init(FaceModelDir);
                            Log.d(TAG, "init: "  + instance.initSeetaFaceSuccess );
                        }
                    }
                    catch (Exception e){
                        instance.isDoingInit = false;
                        instance._isInitFaild = true;
                    }
                }
            }).start();
        }

        return instance;
    }

    /*
     * 检查人脸检测模型文件是否存在
     */
    private static boolean checkModelFilesIsExists() {
        if (!FileUtils.fileIsExists(SeetaFdPath)) {
            Log.e(TAG, "模型不存在，无法检测人脸:" + SeetaFdPath);
            return false;
        }
        if (!FileUtils.fileIsExists(SeetaFaPath)) {
            Log.e(TAG, "模型不存在，无法检测人脸:" + SeetaFaPath);
            return false;
        }
        if (!FileUtils.fileIsExists(SeetaFrPath)) {
            Log.e(TAG, "模型不存在，无法检测人脸:" + SeetaFrPath);
            return false;
        }

        return true;
    }

    public boolean isInited(){
        return this.initSeetaFaceSuccess;
    }

    public boolean isInitFaild(){
        return _isInitFaild && this.initSeetaFaceSuccess == false;
    }

    /*
     * 加载本地人脸图像,并生成人脸特征
     * (后台静默运行)
     */
    private void loadFaceImageFeatures(){

    }


    //初始化so库，告诉底层人脸识别模型文件的目录
    //该目录下应当包括这3个文件：seeta_fd_frontal_v1.0.bin,seeta_fa_v1.1.bin,seeta_fr_v1.0.bin
    public native boolean init(String vModelDir);

    /**
     * 检测人脸
     * @param vImgData：图像数据
     * @param vColNum：图像宽度
     * @param vRowNum：图像高度
     * @param vCh：通道数
     * @param vFaceBmp：人脸抠图（有多个人脸也只抠1一个图返回）
     * @return
     */
    //public native CMSeetaFace[] GetFaces(byte[] vImgData, int vColNum, int vRowNum, int vCh, Bitmap vFaceBmp);

    /**
     * 检测人脸
     * @param vBmp：待检测人脸的大图
     * @param vFaceBmp：其中一个人脸抠图
     * @return
     */
    public native CMSeetaFace[] DetectFaces(Bitmap vBmp, Bitmap vFaceBmp);

    /**
     * 测试
     * @param vVal
     * @return
     */
    public native int Test(int vVal);
    /**
     * 图像的gamma校正
     * @param vColorBmp:原图
     * @param vGammaBmp:处理后的图
     * @param vGamma:gamma值
     */
    public native void imGamma(Bitmap vColorBmp, Bitmap vGammaBmp, float vGamma);

    /**
     * 彩色转灰度图
     * @param vColorBmp
     * @param vGrayBmp
     */
    public native void im2gray(Bitmap vColorBmp, Bitmap vGrayBmp);

    /**
     * 检测人脸，返回各人脸位置，每个人的以;分隔，坐标以分号分隔
     * @param vImgData:图像的char*数据
     * @param vColNum:图像列数
     * @param vRowNum:图像行数
     * @param vCh:图像通道数，3或4
     * @param vDetectModelPath:正面人脸检测模型的绝对路径
     * @param vFaceNo:人脸编号，用于保存特征数据生成文件名用
     * @param vFaceBmp:人脸抠图
     * @return
     */
    public native String DetectFace(byte[] vImgData, int vColNum, int vRowNum, int vCh, String vDetectModelPath, int vFaceNo, Bitmap vFaceBmp);

    /**
     * 比对2个人脸特征值的相似度
     * @param vFeat1
     * @param vFeat2
     * @param vNum
     * @return
     */
    public native float CalcSimilarity(float[] vFeat1, float[] vFeat2);


    /**
     *  提取人脸特征
     * @param vBmp：待检测人脸的大图
     * @return
     */
    public native FaceData[] TakeFacesFeatures(Bitmap vBmp);
}
