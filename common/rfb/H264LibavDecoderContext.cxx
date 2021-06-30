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

#include <rfb/PixelBuffer.h>

extern "C" {
#include <libavutil/imgutils.h>
#include <libavcodec/version.h>
}
#if LIBAVCODEC_VERSION_MAJOR > 57 || LIBAVCODEC_VERSION_MAJOR == 57 && LIBAVCODEC_VERSION_MINOR >= 37
#define FFMPEG_DECODE_VIDEO2_DEPRECATED
#endif

#include <rfb/LogWriter.h>
#include <rfb/H264LibavDecoderContext.h>

using namespace rfb;

static LogWriter vlog("H264LibavDecoderContext");

bool H264LibavDecoderContext::initCodec() {
  sws = NULL;
  swsBuffer = NULL;
  h264AlignedBuffer = NULL;
  h264AlignedCapacity = 0;

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

  int numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGB32, rect.width(), rect.height(), 1);
  swsBuffer = new uint8_t[numBytes];

  initialized = true;
  return true;
}

void H264LibavDecoderContext::freeCodec() {
  if (!initialized)
    return;
  av_parser_close(parser);
  avcodec_free_context(&avctx);
  av_frame_free(&frame);
  delete[] swsBuffer;
  free(h264AlignedBuffer);
  initialized = false;
}

rdr::U8* H264LibavDecoderContext::validateH264BufferLength(rdr::U8* buffer, rdr::U32 len)
{
  rdr::U32 reserve_len = len + len % AV_INPUT_BUFFER_PADDING_SIZE;
  if (len == reserve_len)
    return buffer;

  if (!h264AlignedBuffer || reserve_len > h264AlignedCapacity)
  {
    h264AlignedBuffer = (rdr::U8*)realloc(h264AlignedBuffer, reserve_len);
    h264AlignedCapacity = reserve_len;
  }

  memcpy(h264AlignedBuffer, buffer, len);
  memset(h264AlignedBuffer + len, 0, h264AlignedCapacity - len);
  return h264AlignedBuffer;
}

void H264LibavDecoderContext::decode(rdr::U8* h264_buffer, rdr::U32 len, rdr::U32 flags, ModifiablePixelBuffer* pb) {
  h264_buffer = validateH264BufferLength(h264_buffer, len);

  AVPacket *packet = av_packet_alloc();

  int ret;
  int frames_received = 0;
  while (len)
  {
    ret = av_parser_parse2(parser, avctx, &packet->data, &packet->size, h264_buffer, len, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
    if (ret < 0)
    {
      vlog.error("Error while parsing");
      break;
    }
    // We need to slap on tv to make it work here (don't ask me why)
    if (!packet->size && len == static_cast<rdr::U32>(ret))
      ret = av_parser_parse2(parser, avctx, &packet->data, &packet->size, h264_buffer, len, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
    if (ret < 0)
    {
      vlog.error("Error while parsing");
      break;
    }
    h264_buffer += ret;
    len -= ret;

    if (!ret)
    {
      packet->size = len;
      packet->data = h264_buffer;
      len = 0;
    }

    if (!packet->size)
      continue;

#ifndef FFMPEG_DECODE_VIDEO2_DEPRECATED
    int got_frame;
    ret = avcodec_decode_video2(avctx, frame, &got_frame, packet);
    if (ret < 0 || !got_frame)
    {
      vlog.error("Error during decoding");
      break;
    }
#else
    ret = avcodec_send_packet(avctx, packet);
    if (ret < 0)
    {
      vlog.error("Error sending a packet to decoding");
      break;
    }

    ret = avcodec_receive_frame(avctx, frame);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
      break;
    else if (ret < 0)
    {
      vlog.error("Error during decoding");
      break;
    }
#endif
    frames_received++;

    // vlog.debug("%d frame received", avctx->frame_number);
  }

  packet->size = 0;
  packet->data = NULL;
  av_packet_free(&packet);

  if (!frames_received)
    return;

  if (!frame->height)
    return;

  sws = sws_getCachedContext(sws, frame->width, frame->height, avctx->pix_fmt,
                             frame->width, frame->height, AV_PIX_FMT_RGB32,
                             0, NULL, NULL, NULL);

  int stride;
  pb->getBuffer(rect, &stride);
  int dst_linesize = stride * pb->getPF().bpp/8;  // stride is in pixels, linesize is in bytes (stride x4). We need bytes

  sws_scale(sws, frame->data, frame->linesize, 0, frame->height, &swsBuffer, &dst_linesize);

  pb->imageRect(rect, swsBuffer, stride);
}
