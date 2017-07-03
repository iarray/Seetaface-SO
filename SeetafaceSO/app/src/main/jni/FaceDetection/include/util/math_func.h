/*
 *
 * This file is part of the open-source SeetaFace engine, which includes three modules:
 * SeetaFace Detection, SeetaFace Alignment, and SeetaFace Identification.
 *
 * This file is part of the SeetaFace Detection module, containing codes implementing the
 * face detection method described in the following paper:
 *
 *
 *   Funnel-structured cascade for multi-view face detection with alignment awareness,
 *   Shuzhe Wu, Meina Kan, Zhenliang He, Shiguang Shan, Xilin Chen.
 *   In Neurocomputing (under review)
 *
 *
 * Copyright (C) 2016, Visual Information Processing and Learning (VIPL) group,
 * Institute of Computing Technology, Chinese Academy of Sciences, Beijing, China.
 *
 * The codes are mainly developed by Shuzhe Wu (a Ph.D supervised by Prof. Shiguang Shan)
 *
 * As an open-source face recognition engine: you can redistribute SeetaFace source codes
 * and/or modify it under the terms of the BSD 2-Clause License.
 *
 * You should have received a copy of the BSD 2-Clause License along with the software.
 * If not, see < https://opensource.org/licenses/BSD-2-Clause>.
 *
 * Contact Info: you can send an email to SeetaFace@vipl.ict.ac.cn for any problems.
 *
 * Note: the above information must be kept whenever or wherever the codes are used.
 *
 */

#ifndef SEETA_FD_UTIL_MATH_FUNC_H_
#define SEETA_FD_UTIL_MATH_FUNC_H_

#define WIN_JNI 1
#define ARM_JNI 2

//OS_PLATFORM:
#define JNI_TYPE ARM_JNI

#if (JNI_TYPE - WIN_JNI)
	#include <arm_neon.h>
#else
	#include <immintrin.h>
#endif

#include <cstdint>

namespace seeta {
namespace fd {

class MathFunction {
public:
	static inline void UInt8ToInt32(const uint8_t* src, int32_t* dest,
			int32_t len) {
		for (int32_t i = 0; i < len; i++)
			*(dest++) = static_cast<int32_t>(*(src++));
	}

	static inline void VectorAdd(const int32_t* vSrc1, const int32_t* vSrc2, int32_t* vDst,
			int32_t count) {

#if (JNI_TYPE - WIN_JNI)

		int32_t i;
		for (i = 0; i < count; i += 4)
		{
			int32x4_t in1, in2, out;
			in1 = vld1q_s32(vSrc1);
			vSrc1 += 4;
			in2 = vld1q_s32(vSrc2);
			vSrc2 += 4;

			out = vaddq_s32 (in1, in2);
			vst1q_s32(vDst, out);
			vDst += 4;

			// The following is only an example describing how to use AArch64 specific NEON
			// instructions.
//#if defined (__aarch64__)
//			int tmp = vaddvq_s32(in1);
//#endif

		}
#else
//		    __m128i x1;
//		    __m128i y1;
//		    const __m128i* x2 = reinterpret_cast<const __m128i*>(x);
//		    const __m128i* y2 = reinterpret_cast<const __m128i*>(y);
//		    __m128i* z2 = reinterpret_cast<__m128i*>(z);
//
//		    int32_t i;
//		    for (i = 0; i < len - 4; i += 4) {
//		      x1 = _mm_loadu_si128(x2++);
//		      y1 = _mm_loadu_si128(y2++);
//		      _mm_storeu_si128(z2++, _mm_add_epi32(x1, y1));
//		    }
//		    for (; i < len; i++)
//		      *(z + i) = (*(x + i)) + (*(y + i));
#endif
	}

	static inline void VectorAddC(const int32_t* x, const int32_t* y, int32_t* z,
			int32_t len) {

		for (int32_t i=0; i < len; i++)
			*(z + i) = (*(x + i)) + (*(y + i));
	}


	static inline void VectorSub(const int32_t* vSrc1, const int32_t* vSrc2, int32_t* vDst,
			int32_t count) {
		int32_t i;
		for (i = 0; i < count; i += 4)
		{
			int32x4_t in1, in2, out;
			in1 = vld1q_s32(vSrc1);
			vSrc1 += 4;
			in2 = vld1q_s32(vSrc2);
			vSrc2 += 4;

			out = vsubq_s32 (in1, in2);
			vst1q_s32(vDst, out);
			vDst += 4;
		}

		//	    __m128i x1;
		//	    __m128i y1;
		//	    const __m128i* x2 = reinterpret_cast<const __m128i*>(x);
		//	    const __m128i* y2 = reinterpret_cast<const __m128i*>(y);
		//	    __m128i* z2 = reinterpret_cast<__m128i*>(z);
		//
		//	    int32_t i;
		//	    for (i = 0; i < len - 4; i += 4) {
		//	      x1 = _mm_loadu_si128(x2++);
		//	      y1 = _mm_loadu_si128(y2++);
		//
		//	      _mm_storeu_si128(z2++, _mm_sub_epi32(x1, y1));
		//	    }
		//	    for (; i < len; i++)
		//	      *(z + i) = (*(x + i)) - (*(y + i));
	}

	static inline void VectorSubC(const int32_t* x, const int32_t* y, int32_t* z,
			int32_t len) {

		int32_t i = 0;

		for (; i < len; i++)
			*(z + i) = (*(x + i)) - (*(y + i));
	}
	//
	static inline void VectorAbs(const int32_t* vSrc, int32_t* vDst, int32_t count) {
		int i;
		for (i = 0; i < count; i += 4)
		{
			int32x4_t in1 ;
			in1 = vld1q_s32(vSrc);
			vst1q_s32(vDst, vabsq_s32(in1));
			vSrc += 4;
			vDst += 4;
		}

		//	    __m128i val;
		//	    __m128i val_abs;
		//	    const __m128i* x = reinterpret_cast<const __m128i*>(src);
		//	    __m128i* y = reinterpret_cast<__m128i*>(dest);
		//
		//	    int32_t i;
		//	    for (i = 0; i < len - 4; i += 4) {
		//	      val = _mm_loadu_si128(x++);
		//	      val_abs = _mm_abs_epi32(val);
		//	      _mm_storeu_si128(y++, val_abs);
		//	    }
		//	    for (; i < len; i++)
		//	      dest[i] = (src[i] >= 0 ? src[i] : -src[i]);
	}

	static inline void VectorAbsC(const int32_t* src, int32_t* dest, int32_t len) {

		int32_t i = 0;

		for (; i < len; i++)
			dest[i] = (src[i] >= 0 ? src[i] : -src[i]);
	}

	static inline void Square(const int32_t* vSrc, uint32_t* vDst, int32_t count) {
		int i;

		for (i = 0; i < count; i += 4)
		{
			int32x4_t in1 ;
			in1 = vld1q_s32(vSrc);
			vst1q_u32(vDst, vreinterpretq_u32_s32 (vmulq_s32 (in1, in1)));
			vSrc += 4;
			vDst += 4;
		}

		//    __m128i x1;
		//    const __m128i* x2 = reinterpret_cast<const __m128i*>(src);
		//    __m128i* y2 = reinterpret_cast<__m128i*>(dest);
		//
		//    int32_t i;
		//    for (i = 0; i < len - 4; i += 4) {
		//      x1 = _mm_loadu_si128(x2++);
		//      _mm_storeu_si128(y2++, _mm_mullo_epi32(x1, x1));
		//    }
		//    for (; i < len; i++)
		//      *(dest + i) = (*(src + i)) * (*(src + i));
	}

	static inline void SquareC(const int32_t* src, uint32_t* dest, int32_t len) {

		int32_t i=0;

		for (; i < len; i++)
			*(dest + i) = (*(src + i)) * (*(src + i));
	}
	//
	static inline float VectorInnerProduct(const float* src1, const float* src2, int32_t count)
	{
		int32_t i = 0;
		//neon process
		float32x4_t sum_vec = vdupq_n_f32(0);
		for (; i <count - 3 ; i+=4)
		{
			float32x4_t data_a = vld1q_f32(&src1[i]);
			float32x4_t data_b = vld1q_f32(&src2[i]);
			sum_vec = vaddq_f32(sum_vec, vmulq_f32(data_a, data_b));
		}

		float sum = sum_vec[0] + sum_vec[1] + sum_vec[2] + sum_vec[3];

		//normal process
		for (; i < count; i++)
		{
			sum += src1[i] * src2[i];
		}

		return sum;

		//    __m128 x1;
		//    __m128 y1;
		//    __m128 z1 = _mm_setzero_ps();
		//    float prod;
		//    float buf[4];
		//
		//    int32_t i;
		//    for (i = 0; i < len - 4; i += 4) {
		//      x1 = _mm_loadu_ps(x + i);
		//      y1 = _mm_loadu_ps(y + i);
		//      z1 = _mm_add_ps(z1, _mm_mul_ps(x1, y1));
		//    }
		//    _mm_storeu_ps(&buf[0], z1);
		//    prod = buf[0] + buf[1] + buf[2] + buf[3];
		//    for (; i < len; i++)
		//      prod += x[i] * y[i];
		//
		//    return prod;
	}


	static inline float VectorInnerProduct_c(const float* x, const float* y,
			int32_t len) {

		float prod = 0;

		int32_t i = 0;

		for (; i < len; i++)
			prod += x[i] * y[i];

		return prod;
	}

};

}  // namespace fd
}  // namespace seeta

#endif  // SEETA_FD_UTIL_MATH_FUNC_H_
