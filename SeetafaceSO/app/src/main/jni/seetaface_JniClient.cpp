/**
 * 采用中科院山世光开源的SeetaFaceEngine实现android上的人脸检测与对齐、识别

 * BSD license
 * 广州炒米信息科技有限公司
 * www.cume.cc
 * 吴祖玉
 * wuzuyu365@163.com
 * 2016.11.9
 *
 */

//输出日志开关，如果不需要输出日志，可取消 DEBUG_LOG的定义，该变量在ndk_log.h里面用到
#define DEBUG_LOG

#include "ndk_log.h"
#include "seetaface_JniClient.h"
#include "CMImgProc.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <android/bitmap.h>
#include <sys/time.h>

#include <FaceDetection/include/common.h>
#include <FaceDetection/include/face_detection.h>
#include <FaceAlignment/include/face_alignment.h>
#include <FaceIdentification/include/Face_identification.h>

using namespace seeta;

#ifdef __cplusplus
extern "C" {
#endif

//人脸特征float数组长度
#define	SEETAFACE_FEATURE_NUM 2048

//sdk是否初始化成功
bool sb_sdk_init_ok = false;

//重要：需要java端调用init()来初始化模型路径
seeta::FaceDetection detector(NULL);

//初始化人脸对齐器
//seeta::FaceAlignment point_detector(tAlignModelPath.c_str());
seeta::FaceAlignment point_detector(NULL);
//LOGD("CMDetectFace, point_detector ok");

//seeta::FaceIdentification face_recognizer(tRecoModelPath.c_str());
seeta::FaceIdentification face_recognizer(NULL);

typedef struct
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t alpha;
} rgba;

typedef struct
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} rgb;

/**
 * 初始化seetaface人脸识别库
 * 参数：
 * vFaceModelDir:人脸识别模型文件所在目录，如/sdcard/
 */
JNIEXPORT jboolean JNICALL Java_seetaface_SeetaFace_init(
		JNIEnv *env, jobject obj, jstring vFaceModelDir)
{
	//如果已初始化则直接返回
	if(sb_sdk_init_ok){
		return true;
	}

	jboolean tRet = false;
	if(NULL == vFaceModelDir) {
		return tRet;
	}

	//获取正面人脸检测模型的绝对路径的目录（不是/aaa/bbb.bin这样的路径，是/aaa/)
	const char* tDir = env->GetStringUTFChars(vFaceModelDir, 0);
	if(NULL == tDir) {
		return tRet;
	}
	LOGD("init, tDir=%s", tDir);

	string tFaceModelDir = tDir;
	string tLastChar = tFaceModelDir.substr(tFaceModelDir.length() - 1 , 1);
	LOGD("init, tFaceModelDir last =%s", tLastChar.c_str());

	if("\\" == tLastChar){
		tFaceModelDir = tFaceModelDir.substr(0,tFaceModelDir.length()-1) + "/";
	}else if(tLastChar  != "/"){
		//目录补齐/
		tFaceModelDir += "/";
	}

 	LOGD("init, tFaceModelDir=%s", tFaceModelDir.c_str());

	//检测模型路径
	string tDetectModelPath = tFaceModelDir + "seeta_fd_frontal_v1.0.bin";
	//LOGD("init,  tDetectModelPath=%s", tDetectModelPath.c_str());

	//对齐模型路径
	string tAlignModelPath = tFaceModelDir + "seeta_fa_v1.1.bin";
	//LOGD("init,  tAlignModelPath=%s", tAlignModelPath.c_str());

	//识别模型路径
	string tRecoModelPath = tFaceModelDir + "seeta_fr_v1.0.bin";
	//LOGD("init,  tRecoModelPath=%s", tRecoModelPath.c_str());

	//3个模型文件都要存在，否则初始化失败
	FILE* fp = fopen(tDetectModelPath.c_str(), "r");
	if (!fp){
		LOGE("init, 人脸检测模型不存在, %s", tDetectModelPath.c_str());
		return tRet;
	}
	fclose(fp);

	fp = fopen(tAlignModelPath.c_str(), "r");
	if (!fp){
		LOGE("init, 人脸对齐模型不存在, %s", tAlignModelPath.c_str());
		return tRet;
	}

	fclose(fp);

	fp = fopen(tRecoModelPath.c_str(), "r");
	if (!fp){
		LOGE("init, 人脸识别模型不存在, %s", tRecoModelPath.c_str());
		return tRet;
	}
	fclose(fp);

	LOGD("init, 人脸模型都存在");

	//初始化人脸检测对齐识别器

	detector.initWithModel(tDetectModelPath.c_str());
	point_detector.initWithModel(tAlignModelPath.c_str());
	face_recognizer.initWithModel(tRecoModelPath.c_str());
	sb_sdk_init_ok = true;
	tRet = true;
	return tRet;
}

//获取当前的毫秒时间
long getMillisec(){
	struct timeval tv;
	gettimeofday(&tv,NULL);

	long t0 = tv.tv_sec*1000 + tv.tv_usec/1000;
	return t0;
}

/*
	gamma变换调节亮度,
	用法 jni.imGamma(Bitmap vBmpSrc, Bitmap vBmpDst, float vGamma)
 */
JNIEXPORT void JNICALL Java_seetaface_SeetaFace_imGamma(JNIEnv
		* env, jobject  obj, jobject bitmapcolor1, jobject bitmapcolor2, float vGamma)
{
	LOGI("imGamma");
	if(NULL == bitmapcolor1){
		LOGI("imGamma, bitmapcolor1 is null");
		return;
	}

	if(NULL == bitmapcolor2){
		LOGI("imGamma, bitmapcolor2 is null");
		return;
	}

	AndroidBitmapInfo  infocolor1;
	void*              pixelscolor1;
	AndroidBitmapInfo  infocolor2;
	void*              pixelscolor2;
	int                ret;
	int             y;
	int             x;

	if ((ret = AndroidBitmap_getInfo(env, bitmapcolor1, &infocolor1)) < 0) {
		LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
		return;
	}

	if ((ret = AndroidBitmap_getInfo(env, bitmapcolor2, &infocolor2)) < 0) {
		LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
		return;
	}

//	LOGI("color image 1 :: width is %d; height is %d; stride is %d; format is %d;flags is %d",
//			infocolor1.width,infocolor1.height,infocolor1.stride,infocolor1.format,infocolor1.flags);
	if (infocolor1.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
		LOGE("infocolor1 format is not RGBA_8888 !");
		return;
	}

//	LOGI("color image 2:: width is %d; height is %d; stride is %d; format is %d;flags is %d",
//			infocolor2.width,infocolor2.height,infocolor2.stride,infocolor2.format,infocolor2.flags);
	if (infocolor2.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
		LOGE("infocolor2 format is not RGBA_8888 !");
		return;
	}


	if ((ret = AndroidBitmap_lockPixels(env, bitmapcolor1, &pixelscolor1)) < 0) {
		LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
	}

	if ((ret = AndroidBitmap_lockPixels(env, bitmapcolor2, &pixelscolor2)) < 0) {
		LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
	}

	// modify pixels with image processing algorithm

	float red,green,blue;
	for (y=0;y<infocolor1.height;y++) {
		rgba * line1 = (rgba *) pixelscolor1;
		rgba * line2 = (rgba *) pixelscolor2;
		//uint8_t * line2 = (uint8_t *) pixelscolor2;
		for (x=0; x<infocolor1.width; x++) {

			line2[x].alpha = line1[x].alpha;

			red = (line1[x].red + 0.5F)/256;//归一化
			red = (float)pow(red, vGamma);
			line2[x].red = (uint8_t)(red*256-0.5F);//反归一化

			green = (line1[x].green + 0.5F)/256;//归一化
			green = (float)pow(green, vGamma);
			line2[x].green = (uint8_t)(green*256-0.5F);//反归一化

			blue = (line1[x].blue + 0.5F)/256;//归一化
			blue = (float)pow(blue, vGamma);
			line2[x].blue = (uint8_t)(blue*256-0.5F);//反归一化
		}

		pixelscolor1 = (char *)pixelscolor1 + infocolor1.stride;
		pixelscolor2 = (char *)pixelscolor2 + infocolor2.stride;
	}

//	LOGI("unlocking pixels");
	AndroidBitmap_unlockPixels(env, bitmapcolor1);
	AndroidBitmap_unlockPixels(env, bitmapcolor2);
}

///**
// * 检测人脸
// */
///**
// * jbyteArray v_img_data,图像数据
// * jint cols, 图像宽度
// *  jint rows,图像高度
// *
// */
//JNIEXPORT jobjectArray JNICALL Java_seetaface_SeetaFace_GetFaces(
//		JNIEnv *env, jobject obj, jbyteArray v_img_data,
//		jint cols, jint rows, jint ch, jobject vFaceBmp)
//{
//	if(!sb_sdk_init_ok){
//		LOGE("GetFaces, SDK未初始化");
//		return NULL;
//	}
//
//	LOGD("GetFaces, 1   rows=%d, cols=%d, ch=%d",   rows, cols, ch);
//
//	if(3 == ch || 4 == ch){
//		//图像通道数只能是3或4；
//	}else{
//		return NULL;
//	}
//
//	//获取人脸对象
//	jclass m_cls = env->FindClass("seetaface/CMSeetaFace");
//
//	jmethodID m_mid = env->GetMethodID( m_cls, "<init>", "()V");
//
//	//人脸范围字段
//	jfieldID tLeftField = env->GetFieldID( m_cls, "left", "I");
//	jfieldID tRightField = env->GetFieldID( m_cls, "right", "I");
//	jfieldID tTopField = env->GetFieldID( m_cls, "top", "I");
//	jfieldID tBottomField = env->GetFieldID( m_cls, "bottom", "I");
//	LOGD("GetFaces, 2");
//	//特征点数组,眼睛鼻子嘴巴
//	jfieldID tLandmarkField = env->GetFieldID( m_cls, "landmarks", "[I");
//
//	//特征数组
//	jfieldID tFeaturesField = env->GetFieldID( m_cls, "features", "[F");
//
//	jbyte *tImgData = env->GetByteArrayElements(v_img_data,0);
//
//	//4通道转3通道
//	unsigned char *rgb_bmp3 = new unsigned char[rows*cols*3];
//	LOGD("GetFaces, rgb_bmp3 new ok");
//
//	CMImgProc::RGBA2RGB((unsigned char*)tImgData, rgb_bmp3, cols, rows);
//
//	LOGD("GetFaces, 4通道转3通道 ok");
//
//	unsigned char *gray = new unsigned char[rows*cols];
//	//CMImgProc::RGBA2GRAY((unsigned char*)tImgData, gray, cols, rows, ch);
//	CMImgProc::RGBA2GRAY((unsigned char*)rgb_bmp3, gray, cols, rows, 3);
//	LOGD("GetFaces, 灰度化ok===");
//	//LOGD("tImg, rows=%d, cols=%d",   tImg.rows, tImg.cols);
//
//	ImageData img_color;
//	img_color.data = rgb_bmp3;
//	img_color.width = cols;
//	img_color.height = rows;
//	img_color.num_channels = 3; //因为recognizer要求的是3通道
//
//	//人脸检测要求的是灰度图
//	ImageData img_gray;
//	img_gray.data = gray;
//	img_gray.width = cols;
//	img_gray.height = rows;
//	img_gray.num_channels = 1;
//
//	time_t t0;
//	long tTime0 = getMillisec();
//	//tTime0 = time(&t0);
//
//	LOGD("GetFaces, 当前时间戳:%d", tTime0);
//
//	std::vector<seeta::FaceInfo> faces = detector.Detect(img_gray);
//
//	//LOGD("CMDetectFace, detector.Detect");
//	//time_t t1;
//	long tTime1 = getMillisec(); //time(&t1);
//	LOGD("GetFaces, 检测耗时:%d秒", tTime1 - tTime0);
//
//	int32_t num_face = static_cast<int32_t>(faces.size());
//	if(0 == num_face ){
//		//没有人脸
//		LOGD("GetFaces, 没有人脸");
//		delete[] rgb_bmp3;
//		delete[] gray;
//		return NULL;
//	}else{
//		LOGD("GetFaces, 人脸数:%d", num_face);
//	}
//
//	//	准备裁剪人脸
//	if(NULL != vFaceBmp){
//		AndroidBitmapInfo  infocolor;
//		void*              pixelscolor;
//		int	ret;
//
//		if ((ret = AndroidBitmap_getInfo(env, vFaceBmp, &infocolor)) < 0) {
//			//LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
//		}
//
//		LOGI("vFaceBmp color image :: width is %d; height is %d; stride is %d; format is %d;flags is %d",
//				infocolor.width,infocolor.height,infocolor.stride,infocolor.format,infocolor.flags);
//		if (infocolor.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
//			LOGE("Face Bitmap format is not RGBA_8888 !");
//		}
//
//		if ((ret = AndroidBitmap_lockPixels(env, vFaceBmp, &pixelscolor)) < 0) {
//			LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
//		}
//
//		float feat1[2048];
//		seeta::FacialLandmark face_points_x[5];
//		point_detector.PointDetectLandmarks(img_gray, faces[0], face_points_x);
////		LOGD("CMDetectFace, before ExtractFeatureWithCrop 1");
//
//		int tCropWidth = face_recognizer.crop_width();
//		int tCropHeight = face_recognizer.crop_height();
//		int tCropChannels = face_recognizer.crop_channels();
//
////		LOGD("CMDetectFace, tCropWidth=%d, tCropHeight=%d, tCropChannels=%d" ,
////				tCropWidth, tCropHeight, tCropChannels);
//
//		long tSize = tCropWidth * tCropHeight * tCropChannels;
//		unsigned char *tFaceBytes = new unsigned char[tSize];
//		//剪切出的人脸
//		ImageData img_face;
//		img_face.data = tFaceBytes;
//		img_face.width = face_recognizer.crop_width();
//		img_face.height = face_recognizer.crop_height();
//		img_face.num_channels = face_recognizer.crop_channels();
//
//		//裁剪出人脸
//		int tCropRet = face_recognizer.CropFace(img_color, face_points_x, img_face);
//
////		LOGD("CMDetectFace, tCropRet=%d", tCropRet);
//		int tFaceByteStride = 256*3;
//		int x,y;
//		for (y=0;y<256;y++) {
//			rgb * line1 = (rgb *) tFaceBytes;
//			rgba * line2 = (rgba *) pixelscolor;
//			//uint8_t * line2 = (uint8_t *) pixelscolor2;
//			for (x=0; x<256; x++) {
//				line2[x].alpha = 255;
//				line2[x].red = line1[x].red;
//				line2[x].green = line1[x].green;
//				line2[x].blue = line1[x].blue;
//			}
//			tFaceBytes = (unsigned char *)tFaceBytes + tFaceByteStride;
//			pixelscolor = (unsigned char *)pixelscolor + infocolor.stride;
//		}
//		AndroidBitmap_unlockPixels(env, vFaceBmp);
//		//提取识别用的特征
//	}
//
//	//先建空的数组
//	jobjectArray tRetFaces = env->NewObjectArray(num_face, m_cls, 0);
//	for (int i = 0; i < num_face; i++) {
//		//组装脸部矩形数据
//		seeta::FaceInfo tFace = faces[i];
//
//		//新建一个对象
//		jobject m_obj = env->NewObject( m_cls, m_mid);
//
//		//整型值
//		env->SetIntField( m_obj, tLeftField, tFace.bbox.x);
//		env->SetIntField( m_obj, tRightField, tFace.bbox.x + tFace.bbox.width);
//		env->SetIntField( m_obj, tTopField,  tFace.bbox.y);
//		env->SetIntField( m_obj, tBottomField, tFace.bbox.y + tFace.bbox.width);
//
//		//tLandmarkField
//
//		seeta::FacialLandmark face_points[5];
//		point_detector.PointDetectLandmarks(img_gray, faces[i], face_points);
//
//		//		for(int i=0; i<5; i++){
//		//			sprintf(tPntStr, ",%d,%d", (int) face_points[i].x, (int)face_points[i].y);
//		//
//		//			//strcat(pnts_str, x);
//		//		}
//
//		float feat1[SEETAFACE_FEATURE_NUM];
//		long t1 = getMillisec(); //time(&t1);
//
//		face_recognizer.ExtractFeatureWithCrop(img_color, face_points, feat1);
//		LOGD("GetFaces, ExtractFeatureWithCrop耗时:%d秒",  getMillisec() - t1);
//
//		jfloatArray jnArray = env->NewFloatArray(SEETAFACE_FEATURE_NUM);
//		env->SetFloatArrayRegion(jnArray, 0, SEETAFACE_FEATURE_NUM, feat1);
//		env->SetObjectField(m_obj, tFeaturesField, jnArray);
//
//		/*jfloat nArr[8] = {8,7,6,5,4,3,2,1};
//			jfloatArray jnArray = env->NewFloatArray( 8);
//			env->SetFloatArrayRegion(jnArray,0,8,nArr);
//			env->SetObjectField(m_obj, tFeaturesField,jnArray);
//		 */
//
//		//添加到对象数组
//		env->SetObjectArrayElement(tRetFaces, i, m_obj);
//	}
//
//	delete[] rgb_bmp3;
//	delete[] gray;
//
//	time_t t2;
//	int tTime2 = time(&t2);
//	LOGD("GetFaces, 总耗时:%d秒", tTime2 - tTime0);
//
//	return tRetFaces;
//}


/**
 * 检测人脸
 */
/**
 * jobject vBmp, 待检测人脸图像
 * jobject vFaceBmp，返回其中一个截取的头像，可以为空
 *
 */
JNIEXPORT jobjectArray JNICALL Java_seetaface_SeetaFace_DetectFaces(
		JNIEnv *env, jobject obj, jobject vBmp, jobject vFaceBmp)
{

	LOGD("DetectFaces, 1 ");
	long t0 = getMillisec();
	if(!sb_sdk_init_ok){
		LOGE("DetectFaces, SDK未初始化");
		return NULL;
	}

	if(NULL == vBmp){
		LOGI("DetectFaces, vBmp is null");
		return NULL;
	}

	AndroidBitmapInfo  infocolor1;
	void*              pixelscolor1;
	int ret;
	int             y;
	int             x;

	if ((ret = AndroidBitmap_getInfo(env, vBmp, &infocolor1)) < 0) {
		LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
		return NULL;
	}

//	LOGI("color image 1 :: width is %d; height is %d; stride is %d; format is %d;flags is %d",
//			infocolor1.width,infocolor1.height,infocolor1.stride,infocolor1.format,infocolor1.flags);
	if (infocolor1.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
		LOGE("infocolor1 format is not RGBA_8888 !");
		return NULL;
	}

	if ((ret = AndroidBitmap_lockPixels(env, vBmp, &pixelscolor1)) < 0) {
		LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
		return NULL;
	}

	// modify pixels with image processing algorithm

	int cols = infocolor1.width;
	int rows = infocolor1.height;

	//4通道转3通道
	unsigned char *rgb_bmp3 = new unsigned char[rows*cols*3];
	if(NULL == rgb_bmp3){
		return NULL;
	}
//	LOGD("DetectFaces, rgb_bmp3 new ok");
	unsigned char *pDst = rgb_bmp3;
	float red,green,blue;
//	float tGamma = 0.6f;
//	uint8_t i_red, i_green, i_blue;
	for (y=0;y<infocolor1.height;y++) {
		rgba * line1 = (rgba *) pixelscolor1;
		//uint8_t * line2 = (uint8_t *) pixelscolor2;
		for (x=0; x<infocolor1.width; x++) {
//			red = (line1[x].red + 0.5F)/256;//归一化
//			red = (float)pow(red, tGamma);
//			i_red = (uint8_t)(red*256-0.5F);//反归一化
//
//			green = (line1[x].green + 0.5F)/256;//归一化
//			green = (float)pow(green, tGamma);
//			i_green = (uint8_t)(green*256-0.5F);//反归一化
//
//			blue = (line1[x].blue + 0.5F)/256;//归一化
//			blue = (float)pow(blue, tGamma);
//			i_blue = (uint8_t)(blue*256-0.5F);//反归一化

			*pDst++ = line1[x].red;
			*pDst++ = line1[x].green;
			*pDst++ = line1[x].blue;

//			*pDst++ = i_red;
//			*pDst++ = i_green;
//			*pDst++ = i_blue;
		}

		pixelscolor1 = (char *)pixelscolor1 + infocolor1.stride;
	}

//	LOGI("unlocking pixels");
	AndroidBitmap_unlockPixels(env, vBmp);

	//获取人脸对象
	jclass m_cls = env->FindClass("seetaface/CMSeetaFace");

	jmethodID m_mid = env->GetMethodID( m_cls, "<init>", "()V");

	//人脸范围字段
	jfieldID tLeftField = env->GetFieldID( m_cls, "left", "I");
	jfieldID tRightField = env->GetFieldID( m_cls, "right", "I");
	jfieldID tTopField = env->GetFieldID( m_cls, "top", "I");
	jfieldID tBottomField = env->GetFieldID( m_cls, "bottom", "I");
	LOGD("DetectFaces, 2");
	//特征点数组,眼睛鼻子嘴巴

	//旋转角
	jfieldID tRollField = env->GetFieldID( m_cls, "roll_angle", "F");

	//偏航角
	jfieldID tYawField = env->GetFieldID( m_cls, "yaw_angle", "F");

	//俯仰角
	jfieldID tPitchField = env->GetFieldID( m_cls, "pitch_angle", "F");

	//特征数组
	jfieldID tFeaturesField = env->GetFieldID( m_cls, "features", "[F");

	//特征点数组
	jfieldID tLandmarkField = env->GetFieldID(m_cls, "landmarks", "[I");

	LOGD("DetectFaces, 4通道转3通道 ok");

	unsigned char *gray_bmp = new unsigned char[rows*cols];
	CMImgProc::RGBA2GRAY(rgb_bmp3, gray_bmp, cols, rows, 3);
	LOGD("DetectFaces, 灰度化ok");
	//LOGD("tImg, rows=%d, cols=%d",   tImg.rows, tImg.cols);

	ImageData img_color;
	img_color.data = rgb_bmp3;
	img_color.width = cols;
	img_color.height = rows;
	img_color.num_channels = 3; //因为recognizer要求的是3通道

	//人脸检测要求的是灰度图
	ImageData img_gray;
	img_gray.data = gray_bmp;
	img_gray.width = cols;
	img_gray.height = rows;
	img_gray.num_channels = 1;

	std::vector<seeta::FaceInfo> faces = detector.Detect(img_gray);

	//LOGD("CMDetectFace, detector.Detect");

	long t1 = getMillisec();
	LOGD("DetectFaces, 检测耗时:%ld毫秒", t1 - t0);

	int32_t num_face = static_cast<int32_t>(faces.size());
	if(0 == num_face ){
		//没有人脸
		LOGD("DetectFaces, 没有人脸");
		delete[] rgb_bmp3;
		delete[] gray_bmp;
		return NULL;
	}else{
		LOGD("DetectFaces, 人脸数:%d", num_face);
	}

	//	准备裁剪人脸
	if(NULL != vFaceBmp){
		AndroidBitmapInfo  infocolor;
		void*              pixelscolor;
		int	ret;

		if ((ret = AndroidBitmap_getInfo(env, vFaceBmp, &infocolor)) < 0) {
			//LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
		}

//		LOGI("vFaceBmp color image :: width is %d; height is %d; stride is %d; format is %d;flags is %d",
//				infocolor.width,infocolor.height,infocolor.stride,infocolor.format,infocolor.flags);
		if (infocolor.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
			LOGE("Face Bitmap format is not RGBA_8888 !");
		}

		if ((ret = AndroidBitmap_lockPixels(env, vFaceBmp, &pixelscolor)) < 0) {
			LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
		}

		float feat1[2048];
		seeta::FacialLandmark face_points_x[5];
		point_detector.PointDetectLandmarks(img_gray, faces[0], face_points_x);
//		LOGD("CMDetectFace, before ExtractFeatureWithCrop 1");

		int tCropWidth = face_recognizer.crop_width();
		int tCropHeight = face_recognizer.crop_height();
		int tCropChannels = face_recognizer.crop_channels();

//		LOGD("CMDetectFace, tCropWidth=%d, tCropHeight=%d, tCropChannels=%d" ,
//				tCropWidth, tCropHeight, tCropChannels);

		long tSize = tCropWidth * tCropHeight * tCropChannels;
		unsigned char *tFaceBytes = new unsigned char[tSize];
		//剪切出的人脸
		ImageData img_face;
		img_face.data = tFaceBytes;
		img_face.width = face_recognizer.crop_width();
		img_face.height = face_recognizer.crop_height();
		img_face.num_channels = face_recognizer.crop_channels();

		//裁剪出人脸
		int tCropRet = face_recognizer.CropFace(img_color, face_points_x, img_face);

//		LOGD("CMDetectFace, tCropRet=%d", tCropRet);
		int tFaceByteStride = 256*3;
		int x,y;
		for (y=0;y<256;y++) {
			rgb * line1 = (rgb *) tFaceBytes;
			rgba * line2 = (rgba *) pixelscolor;
			//uint8_t * line2 = (uint8_t *) pixelscolor2;
			for (x=0; x<256; x++) {
				line2[x].alpha = 255;
				line2[x].red = line1[x].red;
				line2[x].green = line1[x].green;
				line2[x].blue = line1[x].blue;
			}
			tFaceBytes = (unsigned char *)tFaceBytes + tFaceByteStride;
			pixelscolor = (unsigned char *)pixelscolor + infocolor.stride;
		}
		AndroidBitmap_unlockPixels(env, vFaceBmp);
		//提取识别用的特征
	}

	//先建空的数组
	jobjectArray tRetFaces = env->NewObjectArray(num_face, m_cls, 0);
	for (int i = 0; i < num_face; i++) {
		//组装脸部矩形数据
		seeta::FaceInfo tFace = faces[i];

		//新建一个对象
		jobject m_obj = env->NewObject( m_cls, m_mid);

		//整型值
		env->SetIntField( m_obj, tLeftField, tFace.bbox.x);
		env->SetIntField( m_obj, tRightField, tFace.bbox.x + tFace.bbox.width);
		env->SetIntField( m_obj, tTopField,  tFace.bbox.y);
		env->SetIntField( m_obj, tBottomField, tFace.bbox.y + tFace.bbox.width);

		//tLandmarkField

		seeta::FacialLandmark face_points[5];
		point_detector.PointDetectLandmarks(img_gray, faces[i], face_points);

		//char tPntStr[200];
		int tLandmarks[10];
		for(int i=0; i<5; i++){
//			sprintf(tPntStr, "landmarks[%d],(%d,%d)", i, (int) face_points[i].x, (int)face_points[i].y);
//			LOGI("特征点:%s", tPntStr);
			tLandmarks[i*2] =  (int) face_points[i].x;
			tLandmarks[i*2+1] =  (int) face_points[i].y;
		}

		//计算roll_angle旋转角
		int dx = face_points[1].x - face_points[0].x;
		int dy = face_points[1].y - face_points[0].y;
		float tRollRad = -atan2(dy, dx);
		float tRollAngle = -atan2(dy, dx)*180/3.1415;
 		env->SetFloatField(m_obj, tRollField, tRollAngle);
		LOGI("旋转角=%lf", tRollAngle);

		//平视角度的5点
		seeta::FacialLandmark rot_face_points[5];
		float tCosRoll = cos(tRollRad);
		float tSinRoll = sin(tRollRad);
//		LOGI("tCosRoll,tSinRoll=(%f,%f)",tCosRoll,tSinRoll);

		//先旋转5点到平视角度
		for(int i=0; i<5; i++){
			rot_face_points[i].x = tCosRoll * face_points[i].x - tSinRoll * face_points[i].y;
			rot_face_points[i].y = tSinRoll * face_points[i].x + tCosRoll * face_points[i].y;
//			LOGI("旋转后点[%d]:(%f,%f)", i, rot_face_points[i].x, rot_face_points[i].y);
		}

		//计算两眼中心点
		float tEyeMidX = 0.5*(rot_face_points[0].x + rot_face_points[1].x);
		//计算鼻尖点与左眼的水平距离
		float tNoseLeftEyeDist = abs(rot_face_points[2].x - rot_face_points[0].x);
		//计算鼻尖点与右眼的水平距离
		float tNoseRightEyeDist = abs(rot_face_points[1].x - rot_face_points[2].x);

		//取鼻尖到2眼的水平距离的最大值
		float tMaxNoseEyeDist = max(tNoseLeftEyeDist, tNoseRightEyeDist);
		//计算偏航角
		float tYawAngle = atan2(tNoseRightEyeDist - tNoseLeftEyeDist, tMaxNoseEyeDist) * 180/3.1415;
		LOGI("偏航角=%f", tYawAngle);
		env->SetFloatField(m_obj, tYawField, tYawAngle);

		//计算俯仰角
		//计算两眼中心点
		float tMouthMidY = 0.5*(rot_face_points[3].y + rot_face_points[4].y);
		//计算鼻尖点与眼睛的竖直距离,这时2个眼睛的y坐标几乎是相同的，所以任取一个眼睛y坐标即可
		float tNoseEyeVertDist = abs(rot_face_points[2].y - rot_face_points[0].y);
		//计算嘴唇与眼的竖直距离
		float tMouthEyeVertDist = abs(tMouthMidY - rot_face_points[0].y);

//		LOGI("tNoseEyeVertDist=%f", tNoseEyeVertDist);
//		LOGI("tMouthEyeVertDist=%f", tMouthEyeVertDist);

		//鼻尖与眼睛距离占眼睛与嘴唇竖直距离的比值
		float tVertDistRatio = tNoseEyeVertDist / (tMouthEyeVertDist + 0.001);
//		LOGI("tVertDistRatio=%f", tVertDistRatio);

		//准备计算俯仰角
		float tVertDistRatioErr = 0;
		if(tVertDistRatio < 0.55){
			tVertDistRatioErr = tVertDistRatio - 0.55;
		}else if (tVertDistRatio > 0.62){
			tVertDistRatioErr = tVertDistRatio - 0.62;
		}

		float tPitchAngle = 2.8 * tVertDistRatioErr * 180/3.1415;
		LOGI("俯仰角=%f", tPitchAngle);
		env->SetFloatField(m_obj, tPitchField, tPitchAngle);

		//特征点
		jintArray jiArray = env->NewIntArray(10);
		env->SetIntArrayRegion(jiArray, 0, 10, tLandmarks);
		env->SetObjectField(m_obj, tLandmarkField, jiArray);

		float feat1[SEETAFACE_FEATURE_NUM];
		long t2 = getMillisec();
		//提取特征
		face_recognizer.ExtractFeatureWithCrop(img_color, face_points, feat1);
		LOGI("提取特征耗时:%ld毫秒", getMillisec() - t2);

		//返回特征
		jfloatArray jnArray = env->NewFloatArray(SEETAFACE_FEATURE_NUM);
		env->SetFloatArrayRegion(jnArray, 0, SEETAFACE_FEATURE_NUM, feat1);
		env->SetObjectField(m_obj, tFeaturesField, jnArray);

		//添加到对象数组
		env->SetObjectArrayElement(tRetFaces, i, m_obj);
	}

	delete[] rgb_bmp3;
	delete[] gray_bmp;

	long t3 = getMillisec();
	LOGD("DetectFaces, 总耗时:%ld毫秒", t3 - t0);

	return tRetFaces;
}

/*
彩色转灰度图
 */
JNIEXPORT void JNICALL Java_seetaface_SeetaFace_im2gray(JNIEnv
		* env, jobject obj, jobject bitmapcolor, jobject bitmapgray)
{
	LOGI("im2gray");
	if(NULL == bitmapcolor){
		LOGI("im2gray, bitmapcolor is null");
		return;
	}

	if(NULL == bitmapgray){
		LOGI("im2gray, bitmapgray is null");
		return;
	}

	AndroidBitmapInfo  infocolor;
	void*              pixelscolor;
	AndroidBitmapInfo  infogray;
	void*              pixelsgray;
	int                ret;
	int             y;
	int             x;

	if ((ret = AndroidBitmap_getInfo(env, bitmapcolor, &infocolor)) < 0) {
		LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
		return;
	}

	if ((ret = AndroidBitmap_getInfo(env, bitmapgray, &infogray)) < 0) {
		LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
		return;
	}

	LOGI("color image :: width is %d; height is %d; stride is %d; format is %d;flags is %d",
			infocolor.width,infocolor.height,infocolor.stride,infocolor.format,infocolor.flags);
	if (infocolor.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
		LOGE("Bitmap format is not RGBA_8888 !");
		return;
	}

	LOGI("gray image :: width is %d; height is %d; stride is %d; format is %d;flags is %d",
			infogray.width,infogray.height,infogray.stride,infogray.format,infogray.flags);
	if (infogray.format != ANDROID_BITMAP_FORMAT_A_8) {
		LOGE("Bitmap format is not A_8 !");
		return;
	}

	if ((ret = AndroidBitmap_lockPixels(env, bitmapcolor, &pixelscolor)) < 0) {
		LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
	}

	if ((ret = AndroidBitmap_lockPixels(env, bitmapgray, &pixelsgray)) < 0) {
		LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
	}

	// modify pixels with image processing algorithm

	for (y=0;y<infocolor.height;y++) {
		rgba * line = (rgba *) pixelscolor;
		uint8_t * grayline = (uint8_t *) pixelsgray;
		for (x=0;x<infocolor.width;x++) {
			grayline[x] = 256 - (0.3 * line[x].red + 0.59 * line[x].green + 0.11*line[x].blue);
		}

		pixelscolor = (char *)pixelscolor + infocolor.stride;
		pixelsgray = (char *) pixelsgray + infogray.stride;
	}

	//LOGI("unlocking pixels");
	AndroidBitmap_unlockPixels(env, bitmapcolor);
	AndroidBitmap_unlockPixels(env, bitmapgray);
}

/*
 *计算2个人脸的相似度
 */
JNIEXPORT jfloat JNICALL Java_seetaface_SeetaFace_CalcSimilarity
(JNIEnv *env, jobject obj, jfloatArray vFeat1, jfloatArray vFeat2)
{
	//LOGD("CalcSimilarity");
	//如果没有初始化则返回
	if(!sb_sdk_init_ok){
		return -1;
	}

	long t0 = getMillisec();

	//LOGD("CalcSimilarity,1");
	if(NULL == vFeat1 || NULL == vFeat2){
		return -1;
	}
	float tSim = -1;

	float *tFeat1, *tFeat2;
	jint i;

	tFeat1 = env->GetFloatArrayElements(vFeat1, 0);
	if (0 == tFeat1) {
		return 0;
	}
	LOGD("CalcSimilarity tFeat1 ok");
	tFeat2 = env->GetFloatArrayElements(vFeat2, 0);
	if (0 == tFeat1) {
		return 0;
	}

	LOGD("CalcSimilarity tFeat2 ok");
	//	LOGD("tFaceModelPath = %s", tFaceModelPath);
	//	LOGD("CalcSimilarity,   vFaceNo1=%d, vFaceNo2=%d",   vFaceNo1, vFaceNo2);

	//	for(int i=0; i<10; i++){
	//		LOGD("CalcSimilarity,feat=%.4f, %.4f", tFeat1[i], tFeat2[i]);
	//	}


	//识别模型路径
//	string tRecoModelPath = tFaceModelPathStr + "seeta_fr_v1.0.bin";
	//	LOGD("CalcSimilarity,  tRecoModelPath=%s", tRecoModelPath.c_str());
	// 	FaceIdentification face_recognizer(tRecoModelPath.c_str());
	//	LOGD("CalcSimilarity, face_recognizer ok");

	//	for(int i=0; i<10; i++){
	//		LOGD("CalcSimilarity, i=%d, feat1=%.4f, %.4f",i, tFeat1[i], tFeat2[i]);
	//	}

	// Caculate similarity of two faces
	tSim = face_recognizer.CalcSimilarity(tFeat1, tFeat2);

	//保留2位小数
	tSim = int(100*tSim)/100.0;
	//LOGD("CalcFaceSim, tSim=%.2f",tSim);
	env->ReleaseFloatArrayElements(vFeat1, tFeat1, 0);
	env->ReleaseFloatArrayElements(vFeat2, tFeat2, 0);

	long t = getMillisec() - t0;
	LOGD("CalcSimilarity, tSim=%.4f, time=%ld", tSim, t);

	return tSim;
}

/*
 * 测试
 */
JNIEXPORT jint JNICALL Java_seetaface_SeetaFace_CMTest
(JNIEnv *env, jobject obj, int vVal)
{
	LOGD("my_thumb");
	//	int x = my_thumb(vVal);
	//	LOGD("my_thumb, x=%d",x);
	//	return x;
	return 0;
}

/*
 *剪切出人脸
 */
JNIEXPORT jbyteArray JNICALL Java_seetaface_SeetaFace_CMCropFace
(JNIEnv *env, jobject obj, jbyteArray v_img_data,
		jint cols, jint rows, jint ch, jstring vFaceModelPath, jobject vFaceBmp)
{
	//	LOGD("CMCropFace, 1");

	AndroidBitmapInfo  infoFaceBmp;

	jbyte *tImgData = env->GetByteArrayElements(v_img_data,0);

	return v_img_data;

	jbyteArray tRetFace = env->NewByteArray(256*256*3);

	//获取正面人脸检测模型的绝对路径
	const char* tFaceModelPath;
	tFaceModelPath = env->GetStringUTFChars(vFaceModelPath, 0);
	if(tFaceModelPath == NULL) {
		return tRetFace;
	}

	//LOGD("tFaceModelPath = %s", tFaceModelPath);
	//LOGD("CMDetectFace,   rows=%d, cols=%d, ch=%d",   rows, cols, ch);

	string tFaceModelPathStr = tFaceModelPath;

	//检测模型路径
	string tDetectModelPath = tFaceModelPathStr + "seeta_fd_frontal_v1.0.bin";
	//LOGD("CMDetectFace,  tDetectModelPath=%s", tDetectModelPath.c_str());

	//对齐模型路径
	string tAlignModelPath = tFaceModelPathStr + "seeta_fa_v1.1.bin";
	//LOGD("CMDetectFace,  tAlignModelPath=%s", tAlignModelPath.c_str());

	//识别模型路径
	string tRecoModelPath = tFaceModelPathStr + "seeta_fr_v1.0.bin";
	//LOGD("CMDetectFace,  tRecoModelPath=%s", tRecoModelPath.c_str());

	//初始化人脸检测器
	seeta::FaceDetection detector(tDetectModelPath.c_str());
	//LOGD("CMDetectFace, detector ok");
	//
	unsigned char *gray = new unsigned char[rows*cols];
	CMImgProc::RGBA2GRAY((unsigned char*)tImgData, gray, cols, rows, ch);
	//LOGD("灰度化ok===");

	//LOGD("tImg, rows=%d, cols=%d",   tImg.rows, tImg.cols);

	ImageData img_data;
	img_data.data =  gray;
	img_data.width = cols;
	img_data.height = rows;
	img_data.num_channels = 1;

	time_t timep;
	time (&timep);
	LOGD("CMDetectFace, 时间1： %s ", ctime(&timep));

	std::vector<seeta::FaceInfo> faces = detector.Detect(img_data);
	//LOGD("CMDetectFace, detector.Detect");
	LOGD("CMDetectFace, 时间2： %s ", ctime(&timep));
	LOGD("CMDetectFace, 时间2");

	int32_t num_face = static_cast<int32_t>(faces.size());
	if(0 == num_face ){
		//没有人脸
		LOGD("CMDetectFace, 没有人脸");
		jstring rtstr = env->NewStringUTF("");
		return tRetFace;
	}else{
		LOGD("CMDetectFace, 人脸数:%d", num_face);
	}

	//初始化人脸对齐器
	seeta::FaceAlignment point_detector(tAlignModelPath.c_str());
	//LOGD("CMDetectFace, point_detector ok");

	std::string tRetFacePosStr = "";
	char tPosStr[200] = {0};

	for(int i=0; i<num_face; i++){
		//每个人脸位置数据用分号分隔
		if(i > 0){
			tRetFacePosStr += ";";
		}

		//组装脸部矩形数据
		seeta::FaceInfo tFace = faces[i];
		sprintf(tPosStr, "%d,%d,%d,%d",
				tFace.bbox.x, tFace.bbox.y, tFace.bbox.width, tFace.bbox.height);

		//LOGD("face_tchar=%s", tPosStr);

		//首先记录人脸位置坐标，4个int
		tRetFacePosStr += tPosStr;

		//然后检测特征点
		seeta::FacialLandmark face_points[5];
		point_detector.PointDetectLandmarks(img_data, faces[i], face_points);
		//组装特征点字符串
		for(int i=0; i<5; i++){
			char tPntStr[100] = {0};
			sprintf(tPntStr, ",%d,%d", (int) face_points[i].x, (int)face_points[i].y);
			tRetFacePosStr += tPntStr;
			//strcat(pnts_str, x);
		}
		//
	}
	LOGD("tRetFacePosStr=%s", tRetFacePosStr.c_str());

	LOGD("CMDetectFace, face_recognizer ok");

	float feat1[2048];
	seeta::FacialLandmark face_points_x[5];
	point_detector.PointDetectLandmarks(img_data, faces[0], face_points_x);
	LOGD("CMDetectFace, before ExtractFeatureWithCrop 1");

	// Create a image to store crop face.
	//剪切出来的头像

	ImageData face_img(face_recognizer.crop_width(), face_recognizer.crop_height(), face_recognizer.crop_channels());

	face_recognizer.CropFace(img_data, face_points_x, face_img);

	LOGD("CMDetectFace, CropFace 1 ok");

	int ret;
	if ((ret = AndroidBitmap_getInfo(env, vFaceBmp, &infoFaceBmp)) < 0) {
		LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
		return tRetFace;
	}
	//LOGD("CMDetectFace, tRetFacePosStr.c_str()=%s", tRetFacePosStr.c_str());

	//sprintf(tRetStr, "face_num=%d", num_face);

	env->ReleaseStringUTFChars(vFaceModelPath, tFaceModelPath);

	return tRetFace;
}


/**
 * 检测人脸
 */
/**
 * jobject vBmp, 待检测人脸图像
 * jobject vFaceBmp，返回其中一个截取的头像，可以为空
 *
 */
JNIEXPORT jobjectArray JNICALL Java_seetaface_SeetaFace_TakeFacesFeatures(
		JNIEnv *env, jobject obj, jobject vBmp)
{

	long t0 = getMillisec();
	if(!sb_sdk_init_ok){
		LOGE("DetectFaces, SDK未初始化");
		return NULL;
	}

	if(NULL == vBmp){
		LOGI("DetectFaces, vBmp is null");
		return NULL;
	}

	AndroidBitmapInfo  infocolor1;
	void*              pixelscolor1;
	int ret;
	int             y;
	int             x;

	if ((ret = AndroidBitmap_getInfo(env, vBmp, &infocolor1)) < 0) {
		LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
		return NULL;
	}


	if (infocolor1.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
		LOGE("infocolor1 format is not RGBA_8888 !");
		return NULL;
	}

	if ((ret = AndroidBitmap_lockPixels(env, vBmp, &pixelscolor1)) < 0) {
		LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
		return NULL;
	}

	// modify pixels with image processing algorithm

	int cols = infocolor1.width;
	int rows = infocolor1.height;

	//4通道转3通道
	unsigned char *rgb_bmp3 = new unsigned char[rows*cols*3];
	if(NULL == rgb_bmp3){
		return NULL;
	}
    
	unsigned char *pDst = rgb_bmp3;
	float red,green,blue;
    
	for (y=0;y<infocolor1.height;y++) {
		rgba * line1 = (rgba *) pixelscolor1;
		for (x=0; x<infocolor1.width; x++) {
			*pDst++ = line1[x].red;
			*pDst++ = line1[x].green;
			*pDst++ = line1[x].blue;
		}

		pixelscolor1 = (char *)pixelscolor1 + infocolor1.stride;
	}

	AndroidBitmap_unlockPixels(env, vBmp);

	//获取人脸对象
	jclass m_cls = env->FindClass("seetaface/FaceData");

	jmethodID m_mid = env->GetMethodID( m_cls, "<init>", "()V");

	//人脸范围字段
	jfieldID tLeftField = env->GetFieldID( m_cls, "left", "I");
	jfieldID tRightField = env->GetFieldID( m_cls, "right", "I");
	jfieldID tTopField = env->GetFieldID( m_cls, "top", "I");
	jfieldID tBottomField = env->GetFieldID( m_cls, "bottom", "I");
	//特征点数组,眼睛鼻子嘴巴
	jfieldID tLandmarkField = env->GetFieldID(m_cls, "landmarks", "[I");
	//特征数组
	jfieldID tFeaturesField = env->GetFieldID( m_cls, "features", "[F");


	LOGD("DetectFaces, 4通道转3通道 ok");

	unsigned char *gray_bmp = new unsigned char[rows*cols];
	CMImgProc::RGBA2GRAY(rgb_bmp3, gray_bmp, cols, rows, 3);
	LOGD("DetectFaces, 灰度化ok");
	

	ImageData img_color;
	img_color.data = rgb_bmp3;
	img_color.width = cols;
	img_color.height = rows;
	img_color.num_channels = 3; //因为recognizer要求的是3通道

	//人脸检测要求的是灰度图
	ImageData img_gray;
	img_gray.data = gray_bmp;
	img_gray.width = cols;
	img_gray.height = rows;
	img_gray.num_channels = 1;

	std::vector<seeta::FaceInfo> faces = detector.Detect(img_gray);

	//LOGD("CMDetectFace, detector.Detect");

	long t1 = getMillisec();
	LOGD("DetectFaces, 检测耗时:%ld毫秒", t1 - t0);

	int32_t num_face = static_cast<int32_t>(faces.size());
	if(0 == num_face ){
		//没有人脸
		LOGD("DetectFaces, 没有人脸");
		delete[] rgb_bmp3;
		delete[] gray_bmp;
		return NULL;
	}else{
		LOGD("DetectFaces, 人脸数:%d", num_face);
	}

	//先建空的数组
	jobjectArray tRetFaces = env->NewObjectArray(num_face, m_cls, 0);
	for (int i = 0; i < num_face; i++) {
		//组装脸部矩形数据
		seeta::FaceInfo tFace = faces[i];

		//新建一个对象
		jobject m_obj = env->NewObject( m_cls, m_mid);

		//整型值
		env->SetIntField( m_obj, tLeftField, tFace.bbox.x);
		env->SetIntField( m_obj, tRightField, tFace.bbox.x + tFace.bbox.width);
		env->SetIntField( m_obj, tTopField,  tFace.bbox.y);
		env->SetIntField( m_obj, tBottomField, tFace.bbox.y + tFace.bbox.width);

		//tLandmarkField
		t1 = getMillisec();
		seeta::FacialLandmark face_points[5];
		point_detector.PointDetectLandmarks(img_gray, faces[i], face_points);
 
		int tLandmarks[10];
		for(int i=0; i<5; i++){
			tLandmarks[i*2] =  (int) face_points[i].x;
			tLandmarks[i*2+1] =  (int) face_points[i].y;
		}
		LOGD("DetectFaces, 五官定位 :%ld毫秒", getMillisec() - t1);


		//特征点
		jintArray jiArray = env->NewIntArray(10);
		env->SetIntArrayRegion(jiArray, 0, 10, tLandmarks);
		env->SetObjectField(m_obj, tLandmarkField, jiArray);

		float feat1[SEETAFACE_FEATURE_NUM];
		long t2 = getMillisec();
		//提取特征
		face_recognizer.ExtractFeatureWithCrop(img_color, face_points, feat1);
		LOGI("提取特征耗时:%ld毫秒", getMillisec() - t2);

		//返回特征
		jfloatArray jnArray = env->NewFloatArray(SEETAFACE_FEATURE_NUM);
		env->SetFloatArrayRegion(jnArray, 0, SEETAFACE_FEATURE_NUM, feat1);
		env->SetObjectField(m_obj, tFeaturesField, jnArray);

		//添加到对象数组
		env->SetObjectArrayElement(tRetFaces, i, m_obj);
	}

	delete[] rgb_bmp3;
	delete[] gray_bmp;

	long t3 = getMillisec();
	LOGD("DetectFaces, 总耗时:%ld毫秒", t3 - t0);

	return tRetFaces;
}

#ifdef __cplusplus
}
#endif
