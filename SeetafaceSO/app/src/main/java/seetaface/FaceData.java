package seetaface;


public class FaceData {

    //人脸范围
    public int left, right, top, bottom;

    //5个点的坐标，眼睛，鼻子，嘴巴
    public int landmarks[] = new int[10];

    //人脸特征
    public float features[] = new float[2048];

}
