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

#include <os/Mutex.h>
#include <rfb/LogWriter.h>

#include <rfb/H264DecoderContext.h>

using namespace rfb;

H264DecoderContext::H264DecoderContext(const Rect &r) : rect(r)
{
  os::AutoMutex lock(&mutex);
  initialized = false;
  delete vlog;
  vlog = new LogWriter("h264 context");

  if (!initCodec())
    return;
  
  vlog->info("Context created");
  initialized = true;
}

H264DecoderContext::~H264DecoderContext() {
  os::AutoMutex lock(&mutex);
  freeCodec();
  initialized = false;
  delete vlog;
}

void H264DecoderContext::reset()
{
  os::AutoMutex lock(&mutex);
  initialized = false;
  freeCodec();
  initialized = initCodec();
}
