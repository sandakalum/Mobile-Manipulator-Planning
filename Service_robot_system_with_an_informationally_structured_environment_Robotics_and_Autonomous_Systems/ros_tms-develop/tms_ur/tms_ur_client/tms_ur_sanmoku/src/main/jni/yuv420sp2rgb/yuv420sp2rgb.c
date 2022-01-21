/*
 * PROJECT: NyARToolkit for Android SDK
 * --------------------------------------------------------------------------------
 * This work is based on the original ARToolKit developed by
 *   Hirokazu Kato
 *   Mark Billinghurst
 *   HITLab, University of Washington, Seattle
 * http://www.hitl.washington.edu/artoolkit/
 *
 * NyARToolkit for Android SDK
 *   Copyright (C)2010 NyARToolkit for Android team
 *   Copyright (C)2010 R.Iizuka(nyatla)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * For further information please contact.
 *  http://sourceforge.jp/projects/nyartoolkit-and/
 *
 * This work is based on the NyARToolKit developed by
 *  R.Iizuka (nyatla)
 *    http://nyatla.jp/nyatoolkit/
 *
 * contributor(s)
 *  Atsuo Igarashi
 */

#include <jni.h>
#include <android/log.h>

#define LOG_TAG "libyuv420sp2rgb"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

/*
 * derived from development/tools/yuv420sp2rgb/yuv420sp2rgb.c
 */
#include <unistd.h>

#ifndef max
#define max(a, b)                                                                                                      \
  (                                                                                                                    \
      {                                                                                                                \
    typeof(a) _a = (a);                                                                                                \
    typeof(b) _b = (b);                                                                                                \
    _a > _b ? _a : _b;                                                                                                 \
      })
#define min(a, b)                                                                                                      \
  (                                                                                                                    \
      {                                                                                                                \
    typeof(a) _a = (a);                                                                                                \
    typeof(b) _b = (b);                                                                                                \
    _a < _b ? _a : _b;                                                                                                 \
      })
#endif

#define CONVERT_TYPE_PPM 0
#define CONVERT_TYPE_RGB 1
#define CONVERT_TYPE_ARGB 2

/*
   YUV 4:2:0 image with a plane of 8 bit Y samples followed by an interleaved
   U/V plane containing 8 bit 2x2 subsampled chroma samples.
   except the interleave order of U and V is reversed.

                        H V
   Y Sample Period      1 1
   U (Cb) Sample Period 2 2
   V (Cr) Sample Period 2 2
 */

typedef struct rgb_context
{
  unsigned char *buffer;
  int width;
  int height;
  int rotate;
  int i;
  int j;
  int size; /* for debugging */
} rgb_context;

typedef void (*rgb_cb)(unsigned char r, unsigned char g, unsigned char b, rgb_context *ctx);

const int bytes_per_pixel = 2;

static void color_convert_common(unsigned char *pY, unsigned char *pUV, int width, int height, unsigned char *buffer,
                                 int size, /* buffer size in bytes */
                                 int gray, int rotate, rgb_cb cb)
{
  int i, j;
  int nR, nG, nB;
  int nY, nU, nV;
  rgb_context ctx;

  ctx.buffer = buffer;
  ctx.size = size; /* debug */
  ctx.width = width;
  ctx.height = height;
  ctx.rotate = rotate;

  if (gray)
  {
    for (i = 0; i < height; i++)
    {
      for (j = 0; j < width; j++)
      {
        nB = *(pY + i * width + j);
        ctx.i = i;
        ctx.j = j;
        cb(nB, nB, nB, &ctx);
      }
    }
  }
  else
  {
    // YUV 4:2:0
    for (i = 0; i < height; i++)
    {
      for (j = 0; j < width; j++)
      {
        nY = *(pY + i * width + j);
        nV = *(pUV + (i / 2) * width + bytes_per_pixel * (j / 2));
        nU = *(pUV + (i / 2) * width + bytes_per_pixel * (j / 2) + 1);

        // Yuv Convert
        nY -= 16;
        nU -= 128;
        nV -= 128;

        if (nY < 0)
          nY = 0;

        // nR = (int)(1.164 * nY + 2.018 * nU);
        // nG = (int)(1.164 * nY - 0.813 * nV - 0.391 * nU);
        // nB = (int)(1.164 * nY + 1.596 * nV);

        nB = (int)(1192 * nY + 2066 * nU);
        nG = (int)(1192 * nY - 833 * nV - 400 * nU);
        nR = (int)(1192 * nY + 1634 * nV);

        nR = min(262143, max(0, nR));
        nG = min(262143, max(0, nG));
        nB = min(262143, max(0, nB));

        nR >>= 10;
        nR &= 0xff;
        nG >>= 10;
        nG &= 0xff;
        nB >>= 10;
        nB &= 0xff;

        ctx.i = i;
        ctx.j = j;
        cb(nR, nG, nB, &ctx);
      }
    }
  }
}

static void common_rgb_cb(unsigned char r, unsigned char g, unsigned char b, rgb_context *ctx, int alpha)
{
  unsigned char *out = ctx->buffer;
  int offset = 0;
  int bpp;
  int i = 0;
  switch (ctx->rotate)
  {
    case 0: /* no rotation */
      offset = ctx->i * ctx->width + ctx->j;
      break;
    case 1: /* 90 degrees */
      offset = ctx->height * (ctx->j + 1) - ctx->i;
      break;
    case 2: /* 180 degrees */
      offset = (ctx->height - 1 - ctx->i) * ctx->width + ctx->j;
      break;
    case 3: /* 270 degrees */
      offset = (ctx->width - 1 - ctx->j) * ctx->height + ctx->i;
      break;
    default:
      // FAILIF(1, "Unexpected roation value %d!\n", ctx->rotate);
      break;
  }

  bpp = 3 + !!alpha;
  offset *= bpp;
  /*FAILIF(offset < 0, "point (%d, %d) generates a negative offset.\n", ctx->i, ctx->j);
  FAILIF(offset + bpp > ctx->size, "point (%d, %d) at offset %d exceeds the size %d of the buffer.\n",
         ctx->i, ctx->j,
         offset,
         ctx->size);*/

  out += offset;

  /*if (alpha) out[i++] = 0xff;
  out[i++] = r;
  out[i++] = g;
  out[i] = b;*/
  out[i++] = b;
  out[i++] = g;
  out[i++] = r;
  if (alpha)
    out[i] = 0xff;
}

static void rgb24_cb(unsigned char r, unsigned char g, unsigned char b, rgb_context *ctx)
{
  return common_rgb_cb(r, g, b, ctx, 0);
}

static void argb_cb(unsigned char r, unsigned char g, unsigned char b, rgb_context *ctx)
{
  return common_rgb_cb(r, g, b, ctx, 1);
}

JNIEXPORT void JNICALL
Java_com_github_irvs_ros_1tms_tms_1ur_tms_1ur_1sanmoku_urclientactivity_ARToolkitDrawer_decodeYUV420SP(
    JNIEnv *env, jobject obj, jintArray rgb, jbyteArray yuv420sp, jint width, jint height, jint type)
{
  void *in, *out;
  int psz = getpagesize();
  int header_size;
  size_t outsize;

  int bpp = 3;
  switch (type)
  {
    case CONVERT_TYPE_RGB:
      header_size = 0;
      break;
    case CONVERT_TYPE_ARGB:
      header_size = 0;
      bpp = 4;
      break;
  }

  outsize = header_size + width * height * bpp;
  outsize = (outsize + psz - 1) & ~(psz - 1);

  jboolean isCopyIn, isCopyOut;
  in = (*env)->GetByteArrayElements(env, yuv420sp, &isCopyIn);
  out = (*env)->GetIntArrayElements(env, rgb, &isCopyOut);

  color_convert_common(in, in + width * height, width, height, out + header_size, outsize - header_size, 0, 0,
                       type == CONVERT_TYPE_ARGB ? argb_cb : rgb24_cb);

  // if (isCopyIn == JNI_TRUE)
  (*env)->ReleaseByteArrayElements(env, yuv420sp, in, 0);
  // if (isCopyOut == JNI_TRUE)
  (*env)->ReleaseIntArrayElements(env, rgb, out, 0);
}
