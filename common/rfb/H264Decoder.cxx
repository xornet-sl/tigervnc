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

#ifdef WIN32
#define WINVER 0x0602
#endif

#include <rdr/Exception.h>
#include <rdr/InStream.h>
#include <rdr/MemInStream.h>
#include <rdr/OutStream.h>
#include <rfb/ServerParams.h>
#include <rfb/PixelBuffer.h>
#include <rfb/LogWriter.h>
#include <rfb/H264Decoder.h>

#ifdef WIN32
#include <mfapi.h>
#include <mferror.h>
#include <wmcodecdsp.h>
#define SAFE_RELEASE(obj) if (obj) { obj->Release(); obj = NULL; }
#else
extern "C" {
#include <libavutil/imgutils.h>
}
#endif

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

#ifndef WIN32
  int numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGB32, r.width(), r.height(), 1);
  swsBuffer = new uint8_t[numBytes];
#endif

  vlog.info("Context created");
  initialized = true;
}

H264DecoderContext::~H264DecoderContext()
{
  os::AutoMutex lock(&mutex);
  _freeCodec();
#ifndef WIN32
  delete[] swsBuffer;
  free(h264AlignedBuffer);
#endif
  initialized = false;
}

bool H264DecoderContext::_initCodec()
{
#ifdef WIN32
  if (FAILED(MFStartup(MF_VERSION, MFSTARTUP_LITE)))
  {
    vlog.error("Could not initialize MediaFoundation");
    return false;
  }

  if (FAILED(CoCreateInstance(CLSID_CMSH264DecoderMFT, NULL, CLSCTX_INPROC_SERVER, IID_IMFTransform, (LPVOID*)&decoder)))
  {
    vlog.error("MediaFoundation H264 codec not found");
    return false;
  }

  GUID CLSID_VideoProcessorMFT = { 0x88753b26, 0x5b24, 0x49bd, { 0xb2, 0xe7, 0xc, 0x44, 0x5c, 0x78, 0xc9, 0x82 } };
  if (FAILED(CoCreateInstance(CLSID_VideoProcessorMFT, NULL, CLSCTX_INPROC_SERVER, IID_IMFTransform, (LPVOID*)&converter)))
  {
    vlog.error("Cannot create MediaFoundation Video Processor (available only on Windows 8+). Trying ColorConvert DMO.");
    if (FAILED(CoCreateInstance(CLSID_CColorConvertDMO, NULL, CLSCTX_INPROC_SERVER, IID_IMFTransform, (LPVOID*)&converter)))
    {
      vlog.error("ColorConvert DMO not found");
      return false;
    }
  }

  // if possible, enable low-latency decoding (Windows 8 and up)
  IMFAttributes* attributes;
  if (SUCCEEDED(decoder->GetAttributes(&attributes)))
  {
    if (SUCCEEDED(attributes->SetUINT32(MF_LOW_LATENCY, TRUE)))
    {
      vlog.info("Enabled low latency mode");
    }
    attributes->Release();
  }

  // set decoder input type
  IMFMediaType* input_type;
  if (FAILED(MFCreateMediaType(&input_type)))
  {
    vlog.error("Could not create MF MediaType");
    return false;
  }
  input_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
  input_type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264);
  decoder->SetInputType(0, input_type, 0);
  input_type->Release();

  // set decoder output type (NV12)
  DWORD output_index = 0;
  IMFMediaType* output_type = NULL;
  while (SUCCEEDED(decoder->GetOutputAvailableType(0, output_index++, &output_type)))
  {
    GUID subtype;
    if (SUCCEEDED(output_type->GetGUID(MF_MT_SUBTYPE, &subtype)) && subtype == MFVideoFormat_NV12)
    {
      decoder->SetOutputType(0, output_type, 0);
      output_type->Release();
      break;
    }
    output_type->Release();
  }

  if (FAILED(decoder->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, 0)))
  {
    vlog.error("Could not start H264 decoder");
    return false;
  }

  MFT_OUTPUT_STREAM_INFO info;
  decoder->GetOutputStreamInfo(0, &info);

  if (FAILED(MFCreateSample(&input_sample)) ||
      FAILED(MFCreateSample(&decoded_sample)) ||
      FAILED(MFCreateSample(&converted_sample)) ||
      FAILED(MFCreateMemoryBuffer(4 * 1024 * 1024, &input_buffer)) ||
      FAILED(MFCreateMemoryBuffer(info.cbSize, &decoded_buffer)))
  {
    vlog.error("Could not allocate media samples/buffers");
    return false;
  }

  input_sample->AddBuffer(input_buffer);
  decoded_sample->AddBuffer(decoded_buffer);

#else

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

#endif
  return true;
}

void H264DecoderContext::_freeCodec()
{
#ifdef WIN32
  SAFE_RELEASE(decoder)
  SAFE_RELEASE(converter)
  SAFE_RELEASE(input_sample)
  SAFE_RELEASE(decoded_sample)
  SAFE_RELEASE(converted_sample)
  SAFE_RELEASE(input_buffer)
  SAFE_RELEASE(decoded_buffer)
  SAFE_RELEASE(converted_buffer)
  MFShutdown();
#else
  av_parser_close(parser);
  avcodec_free_context(&avctx);
  av_frame_free(&frame);
#endif
}

#ifndef WIN32

rdr::U8* H264DecoderContext::validateH264BufferLength(rdr::U8* buffer, rdr::U32 len)
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

#endif

void H264DecoderContext::decode(rdr::U8* h264_buffer, rdr::U32 len, rdr::U32 flags, ModifiablePixelBuffer* pb)
{
  if (!initialized)
    return;

#ifdef WIN32

  if (FAILED(input_buffer->SetCurrentLength(len)))
  {
    input_buffer->Release();
    if (FAILED(MFCreateMemoryBuffer(len, &input_buffer)))
    {
      vlog.error("Could not allocate media buffer");
      return;
    }
    input_buffer->SetCurrentLength(len);
    input_sample->RemoveAllBuffers();
    input_sample->AddBuffer(input_buffer);
  }

  BYTE* locked;
  input_buffer->Lock(&locked, NULL, NULL);
  memcpy(locked, h264_buffer, len);
  input_buffer->Unlock();

  vlog.debug("Received %u bytes, decoding", len);

  if (FAILED(decoder->ProcessInput(0, input_sample, 0)))
  {
    vlog.error("Error sending a packet to decoding");
    return;
  }

  bool decoded = false;

  // try to retrieve all decoded output, as input can submit multiple h264 packets in one buffer
  for (;;)
  {
    decoded_buffer->SetCurrentLength(0);

    MFT_OUTPUT_DATA_BUFFER decoded_data;
    decoded_data.dwStreamID = 0;
    decoded_data.pSample = decoded_sample;
    decoded_data.dwStatus = 0;
    decoded_data.pEvents = NULL;

    DWORD status;
    HRESULT hr = decoder->ProcessOutput(0, 1, &decoded_data, &status);
    SAFE_RELEASE(decoded_data.pEvents)

    if (SUCCEEDED(hr))
    {
      vlog.debug("Frame decoded");
      // successfully decoded next frame
      // but do not exit loop, try again if there is next frame
      decoded = true;
    }
    else if (hr == MF_E_TRANSFORM_NEED_MORE_INPUT)
    {
      // no more frame s to decode
      break;
    }
    else if (hr == MF_E_TRANSFORM_STREAM_CHANGE)
    {
      // something changed (resolution, framerate, h264 properties...)
      // need to setup output type and try decoding again

      DWORD output_index = 0;
      IMFMediaType* output_type = NULL;
      while (SUCCEEDED(decoder->GetOutputAvailableType(0, output_index++, &output_type)))
      {
        GUID subtype;
        if (SUCCEEDED(output_type->GetGUID(MF_MT_SUBTYPE, &subtype)) && subtype == MFVideoFormat_NV12)
        {
          decoder->SetOutputType(0, output_type, 0);
          break;
        }
        output_type->Release();
        output_type = NULL;
      }

      // reinitialize output type (NV12) that now has correct properties (width/height/framerate)
      decoder->SetOutputType(0, output_type, 0);

      UINT32 width = 0;
      UINT32 height = 0;
      MFGetAttributeSize(output_type, MF_MT_FRAME_SIZE, &width, &height);
      vlog.debug("Setting up decoded output with %ux%u size", width, height);

      // input type to converter, BGRX pixel format
      IMFMediaType* converted_type;
      if (FAILED(MFCreateMediaType(&converted_type)))
      {
        vlog.error("Error creating media type");
      }
      else
      {
        converted_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        converted_type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32);
        converted_type->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
        MFSetAttributeSize(converted_type, MF_MT_FRAME_SIZE, width, height);
        MFGetStrideForBitmapInfoHeader(MFVideoFormat_RGB32.Data1, width, &stride);
        // bottom-up
        stride = -stride;
        converted_type->SetUINT32(MF_MT_DEFAULT_STRIDE, (UINT32)stride);

        // setup NV12 -> BGRX converter
        converter->SetOutputType(0, converted_type, 0);
        converter->SetInputType(0, output_type, 0);
        converted_type->Release();

        // create converter output buffer

        MFT_OUTPUT_STREAM_INFO info;
        converter->GetOutputStreamInfo(0, &info);

        if (FAILED(MFCreateMemoryBuffer(info.cbSize, &converted_buffer)))
        {
          vlog.error("Error creating media buffer");
        }
        else
        {
          converted_sample->AddBuffer(converted_buffer);
        }
      }
      output_type->Release();
    }
  }

  // we care only about final image
  // we ignore previous images if decoded multiple in a row
  if (decoded)
  {
    if (FAILED(converter->ProcessInput(0, decoded_sample, 0)))
    {
      vlog.error("Error sending a packet to converter");
      return;
    }

    MFT_OUTPUT_DATA_BUFFER converted_data;
    converted_data.dwStreamID = 0;
    converted_data.pSample = converted_sample;
    converted_data.dwStatus = 0;
    converted_data.pEvents = NULL;

    DWORD status;
    HRESULT hr = converter->ProcessOutput(0, 1, &converted_data, &status);
    SAFE_RELEASE(converted_data.pEvents)

    if (FAILED(hr))
    {
      vlog.error("Error during conversion");
    }
    else
    {
      vlog.debug("Frame converted to RGB");

      BYTE* out;
      converted_buffer->Lock(&out, NULL, NULL);
      pb->imageRect(rect, out, (int)stride / 4);
      converted_buffer->Unlock();
    }
  }

#else
  
  h264_buffer = validateH264BufferLength(h264_buffer, len);

  AVPacket packet;
  av_init_packet(&packet);

  int ret;
  int frames_received = 0;
  while (len)
  {
    ret = av_parser_parse2(parser, avctx, &packet.data, &packet.size, h264_buffer, len, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
    if (ret < 0)
    {
      vlog.error("Error while parsing");
      break;
    }
    // We need to slap on tv to make it work here (don't ask me why)
    if (!packet.size && len == static_cast<rdr::U32>(ret))
      ret = av_parser_parse2(parser, avctx, &packet.data, &packet.size, h264_buffer, len, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
    if (ret < 0)
    {
      vlog.error("Error while parsing");
      break;
    }
    h264_buffer += ret;
    len -= ret;

    if (!ret)
    {
      packet.size = len;
      packet.data = h264_buffer;
      len = 0;
    }

    if (!packet.size)
      continue;

    ret = avcodec_send_packet(avctx, &packet);
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
    frames_received++;

    // vlog.debug("%d frame received", avctx->frame_number);
  }

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

#endif
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

  ctx->decode(h264_buffer, len, flags, pb);
}
