/*
 *   This file is provided under a GPLv2 license.  When using or
 *   redistributing this file, you may do so under that license.
 *
 *   GPL LICENSE SUMMARY
 *
 *   Copyright (C) 2017 T-Platforms All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or modify it
 *   under the terms and conditions of the GNU General Public License,
 *   version 2, as published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 *   more details.
 *
 *   You should have received a copy of the GNU General Public License along with
 *   this program; if not, one can be found <http://www.gnu.org/licenses/>.
 *
 *   The full GNU General Public License is included in this distribution in
 *   the file called "COPYING".
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * NTB test drivers stub
 *
 * Contact Information:
 * Serge Semin <fancer.lancer@gmail.com>, <Sergey.Semin@t-platforms.ru>
 */

#include <linux/kernel.h>
#include <linux/dmaengine.h>

/*
 * bitmap_from_u64 - Check and swap words within u64.
 *  @mask: source bitmap
 *  @dst:  destination bitmap
 *
 * In 32-bit Big Endian kernel, when using (u32 *)(&val)[*]
 * to read u64 mask, we will get the wrong word.
 * That is "(u32 *)(&val)[0]" gets the upper 32 bits,
 * but we expect the lower 32-bits of u64.
 */
static inline void bitmap_from_u64(unsigned long *dst, u64 mask)
{
	dst[0] = mask & ULONG_MAX;

	if (sizeof(mask) > sizeof(unsigned long))
		dst[1] = mask >> 32;
}

static inline struct dma_async_tx_descriptor *dmaengine_prep_dma_memcpy(
		struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
		size_t len, unsigned long flags)
{
	if (!chan || !chan->device || !chan->device->device_prep_dma_memcpy)
		return NULL;

	return chan->device->device_prep_dma_memcpy(chan, dest, src,
						    len, flags);
}

/**
 * dmaengine_terminate_async() - Terminate all active DMA transfers
 * @chan: The channel for which to terminate the transfers
 *
 * Calling this function will terminate all active and pending descriptors
 * that have previously been submitted to the channel. It is not guaranteed
 * though that the transfer for the active descriptor has stopped when the
 * function returns. Furthermore it is possible the complete callback of a
 * submitted transfer is still running when this function returns.
 *
 * dmaengine_synchronize() needs to be called before it is safe to free
 * any memory that is accessed by previously submitted descriptors or before
 * freeing any resources accessed from within the completion callback of any
 * perviously submitted descriptors.
 *
 * This function can be called from atomic context as well as from within a
 * complete callback of a descriptor submitted on the same channel.
 *
 * If none of the two conditions above apply consider using
 * dmaengine_terminate_sync() instead.
 */
static inline int dmaengine_terminate_async(struct dma_chan *chan)
{
	if (chan->device->device_terminate_all)
		return chan->device->device_terminate_all(chan);

	return -EINVAL;
}

/**
 * dmaengine_synchronize() - Synchronize DMA channel termination
 * @chan: The channel to synchronize
 *
 * Synchronizes to the DMA channel termination to the current context. When this
 * function returns it is guaranteed that all transfers for previously issued
 * descriptors have stopped and and it is safe to free the memory assoicated
 * with them. Furthermore it is guaranteed that all complete callback functions
 * for a previously submitted descriptor have finished running and it is safe to
 * free resources accessed from within the complete callbacks.
 *
 * The behavior of this function is undefined if dma_async_issue_pending() has
 * been called between dmaengine_terminate_async() and this function.
 *
 * This function must only be called from non-atomic context and must not be
 * called from within a complete callback of a descriptor submitted on the same
 * channel.
 */
static inline void dmaengine_synchronize(struct dma_chan *chan)
{
	might_sleep();

	/*if (chan->device->device_synchronize)
		chan->device->device_synchronize(chan);
	 */
}

/**
 * dmaengine_terminate_sync() - Terminate all active DMA transfers
 * @chan: The channel for which to terminate the transfers
 *
 * Calling this function will terminate all active and pending transfers
 * that have previously been submitted to the channel. It is similar to
 * dmaengine_terminate_async() but guarantees that the DMA transfer has actually
 * stopped and that all complete callbacks have finished running when the
 * function returns.
 *
 * This function must only be called from non-atomic context and must not be
 * called from within a complete callback of a descriptor submitted on the same
 * channel.
 */
static inline int dmaengine_terminate_sync(struct dma_chan *chan)
{
	int ret;

	ret = dmaengine_terminate_async(chan);
	if (ret)
		return ret;

	dmaengine_synchronize(chan);

	return 0;
}

