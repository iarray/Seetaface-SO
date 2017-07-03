首先,打开Android SDK 管理工具下载好NDK

例如下载到: D:\Android\sdk\ndk-bundle

打开命令行窗口, 进入到目录 SeetafaceSO\app\src\main

在目录下执行 D:\Android\sdk\ndk-bundle\ndk-build 即可编译

不同版本的编译参数都不同 

* 编译任何版本都需要 -fstrict-aliasing

* 编译 arm64-v8a 和 x86 版本只保留-fstrict-aliasing 其它都删掉

* 编译 armeabi 版本请加上 -mfloat-abi=softfp -mfpu=neon

* 编译 armeabi-v7a 版本请加上 -fprefetch-loop-arrays -mfpu=neon

注意在 SeetafaceSO\app\src\main\jni\Application.mk 文件中的参数 NDK_TOOLCHAIN_VERSION=4.9

版本号要和NDK目录下的toolchains文件夹内的文件名的数字版本一致,本例子中的路径是:
D:\Android\sdk\ndk-bundle\toolchains


libs目录提供了我编译好的各个平台的版本,注意这个编译好的版本没有下面这个方法

 public native FaceData[] TakeFacesFeatures(Bitmap vBmp);
 
这个方法是我后面加上去的,但我没有为其编译多个版本, 其实就是DetectFaces方法里面的一些我用不到的剔掉了