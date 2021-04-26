/* Copyright (C) 2021 Vladimir Sukhonosov <xornet@xornet.org>
 * Copyright (C) 2021 Martins Mozeiko <martins.mozeiko@gmail.com>
 * All Rights Reserved.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307,
 * USA.
 */

#ifndef __RFB_H264DECODER_H__
#define __RFB_H264DECODER_H__

#include <deque>
#include <vector>

#ifdef WIN32
#include <windows.h>
#include <mfidl.h>
#include <mftransform.h>
#else
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/version.h>
#include <libswscale/swscale.h>
}
#if LIBAVCODEC_VERSION_MAJOR > 57 || LIBAVCODEC_VERSION_MAJOR == 57 && LIBAVCODEC_VERSION_MINOR >= 37
#define FFMPEG_DECODE_VIDEO2_DEPRECATED
#endif
#endif

#include <os/Mutex.h>
#include <rfb/Decoder.h>

namespace rfb {
  struct H264DecoderContext {
    public:
      H264DecoderContext(const Rect& r);
      ~H264DecoderContext();

      void decode(rdr::U8* h264_buffer, rdr::U32 len, rdr::U32 flags, ModifiablePixelBuffer* pb);
      void reset();
      inline bool isEqualRect(const Rect& r) const { return 0 == memcmp(&rect, &r, sizeof(Rect)); }
      inline bool isReady() const { return initialized; }

      os::Mutex mutex;

    private:
      bool _initCodec();
      void _freeCodec();
      rdr::U8* validateH264BufferLength(rdr::U8* buffer, rdr::U32 len);

      Rect rect;
#ifdef WIN32
      LONG stride;
      IMFTransform *decoder = NULL;
      IMFTransform *converter = NULL;
      IMFSample *input_sample = NULL;
      IMFSample *decoded_sample = NULL;
      IMFSample *converted_sample = NULL;
      IMFMediaBuffer *input_buffer = NULL;
      IMFMediaBuffer *decoded_buffer = NULL;
      IMFMediaBuffer *converted_buffer = NULL;
#else
      AVCodecContext *avctx;
      AVCodecParserContext *parser;
      AVFrame* frame;
      SwsContext* sws;
      uint8_t* swsBuffer;
      rdr::U8* h264AlignedBuffer;
      rdr::U32 h264AlignedCapacity;
#endif
      bool initialized;
  };

  class H264Decoder : public Decoder {
  public:
    H264Decoder();
    virtual ~H264Decoder();
    virtual bool readRect(const Rect& r, rdr::InStream* is,
                          const ServerParams& server, rdr::OutStream* os);
    virtual void decodeRect(const Rect& r, const void* buffer,
                            size_t buflen, const ServerParams& server,
                            ModifiablePixelBuffer* pb);

  private:
    void resetContexts();
    H264DecoderContext* findContext(const Rect& r, bool lock = false);

    os::Mutex mutex;
    std::deque<H264DecoderContext*> contexts;
  };
}


#endif
