# 查找源文件, 定义本地文件所在的目录路径
LOCAL_PATH := $(call my-dir)
# CLEAR_VARS由编译系统提供,指定让GNU MAKEFILE清除LOCAL_PATH
# 以外的所有LOCAL_XXX变量

include $(CLEAR_VARS)

#无需opencv
#include D:\workspace\OpenCV-android-sdk\sdk\native\jni\opencv.mk
#OPENCV_INSTALL_MODULES:=on
#OPENCV_CAMERA_MODULES:=off 
#OPENCV_LIB_TYPE:=STATIC

#编译任何版本都需要 -fstrict-aliasing
#编译 arm64-v8a 和 x86 版本只保留-fstrict-aliasing 其它都删掉
#编译 armeabi 版本请加上 -mfloat-abi=softfp -mfpu=neon
#编译 armeabi-v7a 版本请加上 -fprefetch-loop-arrays -mfpu=neon

LOCAL_MODULE := SeetafaceSo  
LOCAL_LDLIBS += -llog -ljnigraphics  
LOCAL_CFLAGS += -O2 -fstrict-aliasing -fprefetch-loop-arrays -mfloat-abi=softfp -mfpu=neon

# 编译 armeabi 版本, 请取消注释下面三行代码
TARGET_ARCH_ABI :=armeabi-v7a
LOCAL_ARM_MODE := arm    
LOCAL_ARM_NEON := true 

#使用NEON优化
LOCAL_CFLAGS += -DHAVE_NEON=1

LOCAL_C_INCLUDES := $(LOCAL_PATH)/FaceDetection/include \
$(LOCAL_PATH)/FaceIdentification/include \
$(LOCAL_PATH)/FaceAlignment/include  
				 					  	   		  	  
LOCAL_SRC_FILES := FaceDetection/src/face_detection.cpp \
FaceDetection/src/fust.cpp \
FaceDetection/src/util/image_pyramid.cpp \
FaceDetection/src/util/nms.cpp \
FaceDetection/src/feat/lab_feature_map.cpp \
FaceDetection/src/feat/surf_feature_map.cpp \
FaceDetection/src/io/lab_boost_model_reader.cpp \
FaceDetection/src/io/surf_mlp_model_reader.cpp \
FaceDetection/src/classifier/lab_boosted_classifier.cpp \
FaceDetection/src/classifier/mlp.cpp   \
FaceDetection/src/classifier/surf_mlp.cpp \
FaceAlignment/cfan.cpp  \
FaceAlignment/face_alignment.cpp \
FaceAlignment/sift.cpp \
FaceIdentification\src\bias_adder_net.cpp \
FaceIdentification\src\blob.cpp \
FaceIdentification\src\bn_net.cpp \
FaceIdentification\src\common_net.cpp \
FaceIdentification\src\conv_net.cpp \
FaceIdentification\src\eltwise_net.cpp \
FaceIdentification\src\inner_product_net.cpp \
FaceIdentification\src\log.cpp \
FaceIdentification\src\math_functions.cpp \
FaceIdentification\src\max_pooling_net.cpp \
FaceIdentification\src\net.cpp \
FaceIdentification\src\pad_net.cpp \
FaceIdentification\src\spatial_transform_net.cpp \
FaceIdentification\src\tform_maker_net.cpp \
FaceIdentification\tools\aligner.cpp \
FaceIdentification\tools\face_identification.cpp \
Seetaface_JniClient.cpp \
CMImgProc.cpp 
    
include $(BUILD_SHARED_LIBRARY)
      