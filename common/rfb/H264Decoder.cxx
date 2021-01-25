/* Copyright (C) 2021 Vladimir Sukhonosov <xornet@xornet.org>
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

extern "C" {
#include <libavutil/imgutils.h>
}

#include <rdr/Exception.h>
#include <rdr/InStream.h>
#include <rdr/MemInStream.h>
#include <rdr/OutStream.h>
#include <rfb/ServerParams.h>
#include <rfb/PixelBuffer.h>
#include <rfb/LogWriter.h>
#include <rfb/H264Decoder.h>

using namespace rfb;

#define MAX_H264_INSTANCES 255

enum rectFlags {
  resetContext       = 0x1,
  resetAllContexts   = 0x2,
};

static LogWriter vlog("h264");

H264DecoderContext::H264DecoderContext(const Rect& r) : rect(r)
{
  os::AutoMutex lock(&mutex);

  if (!_initCodec())
    return;

  int numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGB32, r.width(), r.height(), 1);
  sws_buffer = new uint8_t[numBytes];

  vlog.info("Context created");
  initialized = true;
}

H264DecoderContext::~H264DecoderContext()
{
  os::AutoMutex lock(&mutex);
  _freeCodec();
  delete[] sws_buffer;
  initialized = false;
}

bool H264DecoderContext::_initCodec()
{
  AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec)
  {
    vlog.error("Codec not found");
    return false;
  }

  parser = av_parser_init(codec->id);
  if (!parser)
  {
    vlog.error("Could not create H264 parser");
    return false;
  }

  avctx = avcodec_alloc_context3(codec);
  if (!avctx)
  {
    vlog.error("Could not allocate video codec context");
    return false;
  }

  frame = av_frame_alloc();
  if (!frame)
  {
    vlog.error("Could not allocate video frame");
    return false;
  }

  if (avcodec_open2(avctx, codec, NULL) < 0)
  {
    vlog.error("Could not open codec");
    return false;
  }
  return true;
}

void H264DecoderContext::_freeCodec()
{
  av_parser_close(parser);
  avcodec_free_context(&avctx);
  av_frame_free(&frame);
}

void H264DecoderContext::decode(rdr::U8* h264_buffer, rdr::U32 len, rdr::U32 flags, ModifiablePixelBuffer* pb)
{
  if (!initialized)
    return;
  AVPacket packet;
  av_init_packet(&packet);
  packet.size = len;
  packet.data = h264_buffer;

  int ret = avcodec_send_packet(avctx, &packet);
  if (ret < 0)
  {
    vlog.error("Error sending a packet to decoding");
    return;
  }

  ret = avcodec_receive_frame(avctx, frame);
  if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
    return;
  else if (ret < 0)
  {
    vlog.error("Error during decoding");
    return;
  }

  // vlog.debug("%d frame received", avctx->frame_number);

  if (!frame->height)
    return;

  sws = sws_getCachedContext(sws, frame->width, frame->height, avctx->pix_fmt,
                             frame->width, frame->height, AV_PIX_FMT_RGB32,
                             0, NULL, NULL, NULL);
  int stride;
  pb->getBuffer(rect, &stride);
  int dst_linesize = stride * pb->getPF().bpp/8;  // stride is in pixels, linesize is in bytes (stride x4). We need bytes

  sws_scale(sws, frame->data, frame->linesize, 0, frame->height, &sws_buffer, &dst_linesize);

  pb->imageRect(rect, sws_buffer, stride);
}

void H264DecoderContext::reset()
{
  os::AutoMutex lock(&mutex);
  initialized = false;
  _freeCodec();
  initialized = _initCodec();
}


H264Decoder::H264Decoder() : Decoder(DecoderPlain)
{
}

H264Decoder::~H264Decoder()
{
  resetContexts();
}

void H264Decoder::resetContexts()
{
  os::AutoMutex lock(&mutex);
  for (std::deque<H264DecoderContext*>::iterator it = contexts.begin(); it != contexts.end(); it++)
    delete *it;
  contexts.clear();
}

H264DecoderContext* H264Decoder::findContext(const Rect& r, bool lock)
{
  os::AutoMutex m(&mutex);
  for (std::deque<H264DecoderContext*>::iterator it = contexts.begin(); it != contexts.end(); it++)
    if ((*it)->isEqualRect(r))
    {
      if (lock)
        (*it)->mutex.lock();
      return *it;
    }
  return NULL;
}

bool H264Decoder::readRect(const Rect& r, rdr::InStream* is,
                                const ServerParams& server, rdr::OutStream* os)
{
  rdr::U32 len;

  if (!is->hasData(8))
    return false;

  is->setRestorePoint();

  len = is->readU32();
  os->writeU32(len);
  rdr::U32 flags = is->readU32();

  if (flags & rectFlags::resetAllContexts)
  {
    resetContexts();
    if (!len)
      return false;
    flags &= ~(rectFlags::resetContext | rectFlags::resetAllContexts);
  }
  os->writeU32(flags);

  if (!is->hasDataOrRestore(len))
    return false;

  is->clearRestorePoint();

  os->copyBytes(is, len);

  os::AutoMutex lock(&mutex);
  if (!findContext(r))
  {
    if (contexts.size() >= MAX_H264_INSTANCES)
    {
      delete contexts.front();
      contexts.pop_front();
    }
    contexts.push_back(new H264DecoderContext(r));
  }

  return true;
}

void H264Decoder::decodeRect(const Rect& r, const void* buffer,
                             size_t buflen, const ServerParams& server,
                             ModifiablePixelBuffer* pb)
{
  H264DecoderContext* ctx = findContext(r, true);
  if (!ctx)
  {
    vlog.error("Context not found for decoding rect");
    return;
  }
  os::AutoMutex lock(&ctx->mutex, true);
  if (!ctx->isReady())
    return;
  rdr::MemInStream is(buffer, buflen);
  rdr::U32 len = is.readU32();
  rdr::U32 flags = is.readU32();

  if (flags & rectFlags::resetContext)
    ctx->reset();

  if (!len)
    return;

  rdr::U8* h264_buffer = const_cast<rdr::U8*>(is.getptr(0));

  // Check if buffer wasn't well padded
  bool bufferReallocated = false;
  if (len % AV_INPUT_BUFFER_PADDING_SIZE)
  {
    // bad, bad encoder
    vlog.debug("bad buffer padding, wastefull memory reallocation");
    h264_buffer = new rdr::U8[len + len % AV_INPUT_BUFFER_PADDING_SIZE];
    memcpy(h264_buffer, is.getptr(0), len);
    memset(h264_buffer + len, 0, len % AV_INPUT_BUFFER_PADDING_SIZE);
    bufferReallocated = true;
  }

  ctx->decode(h264_buffer, len, flags, pb);
  if (bufferReallocated)
  {
    delete[] h264_buffer;
    h264_buffer = NULL;
  }
}
