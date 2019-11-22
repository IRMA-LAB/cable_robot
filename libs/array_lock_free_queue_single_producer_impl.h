// ============================================================================
// Copyright (c) 2010 Faustino Frechilla
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//  2. Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//  3. The name of the author may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//
/// @file array_lock_free_queue_single_producer_impl.h
/// @brief Implementation of a circular array based lock-free queue
///
/// @author Faustino Frechilla
/// @history
/// Ref  Who                 When         What
///      Faustino Frechilla  11-Jul-2010  Original development
/// @endhistory
/// 
// ============================================================================

#ifndef __ARRAY_LOCK_FREE_QUEUE_SINGLE_PRODUCER_IMPL_H__
#define __ARRAY_LOCK_FREE_QUEUE_SINGLE_PRODUCER_IMPL_H__

#include <assert.h> // assert()

template <typename ELEM_T, uint32_t Q_SIZE>
ArrayLockFreeQueueSingleProducer<ELEM_T, Q_SIZE>::ArrayLockFreeQueueSingleProducer() :
    m_writeIndex(0),
    m_readIndex(0)
#ifdef ARRAY_LOCK_FREE_Q_KEEP_REAL_SIZE
    ,m_count(0)
#endif
{
}

template <typename ELEM_T, uint32_t Q_SIZE>
ArrayLockFreeQueueSingleProducer<ELEM_T, Q_SIZE>::~ArrayLockFreeQueueSingleProducer()
{
}

template <typename ELEM_T, uint32_t Q_SIZE>
inline
uint32_t ArrayLockFreeQueueSingleProducer<ELEM_T, Q_SIZE>::countToIndex(uint32_t a_count)
{
    // if Q_SIZE is a power of 2 this statement could be also written as 
    // return (a_count & (Q_SIZE - 1));
    return (a_count % Q_SIZE);
}

template <typename ELEM_T, uint32_t Q_SIZE>
uint32_t ArrayLockFreeQueueSingleProducer<ELEM_T, Q_SIZE>::size()
{
#ifdef ARRAY_LOCK_FREE_Q_KEEP_REAL_SIZE
    return m_count;
#else
    uint32_t currentWriteIndex = m_writeIndex;
    uint32_t currentReadIndex  = m_readIndex;

    // let's think of a scenario where this function returns bogus data
    // 1. when the statement 'currentWriteIndex = m_writeIndex' is run
    // m_writeIndex is 3 and m_readIndex is 2. Real size is 1
    // 2. afterwards this thread is preemted. While this thread is inactive 2 
    // elements are inserted and removed from the queue, so m_writeIndex is 5
    // m_readIndex 4. Real size is still 1
    // 3. Now the current thread comes back from preemption and reads m_readIndex.
    // currentReadIndex is 4
    // 4. currentReadIndex is bigger than currentWriteIndex, so
    // m_totalSize + currentWriteIndex - currentReadIndex is returned, that is,
    // it returns that the queue is almost full, when it is almost empty
    
    if (currentWriteIndex >= currentReadIndex)
    {
        return (currentWriteIndex - currentReadIndex);
    }
    else
    {
        return (Q_SIZE + currentWriteIndex - currentReadIndex);
    }
#endif // ARRAY_LOCK_FREE_Q_KEEP_REAL_SIZE
}

template <typename ELEM_T, uint32_t Q_SIZE>
bool ArrayLockFreeQueueSingleProducer<ELEM_T, Q_SIZE>::push(const ELEM_T &a_data)
{
    uint32_t currentReadIndex;
    uint32_t currentWriteIndex;

    currentWriteIndex = m_writeIndex;
    currentReadIndex  = m_readIndex;
    if (countToIndex(currentWriteIndex + 1) == 
        countToIndex(currentReadIndex))
    {
        // the queue is full
        return false;
    }

    // save the date into the q
    m_theQueue[countToIndex(currentWriteIndex)] = a_data;
    
    // No need to increment write index atomically. It is a 
    // a requierement of this queue that only one thred can push stuff in
    m_writeIndex++;

    // The value was successfully inserted into the queue
#ifdef ARRAY_LOCK_FREE_Q_KEEP_REAL_SIZE
    AtomicAdd(&m_count, 1);
#endif

    return true;
}

template <typename ELEM_T, uint32_t Q_SIZE>
bool ArrayLockFreeQueueSingleProducer<ELEM_T, Q_SIZE>::pop(ELEM_T &a_data)
{
    uint32_t currentMaximumReadIndex;
    uint32_t currentReadIndex;

    do
    {
        // m_maximumReadIndex doesn't exist when the queue is set up as
        // single-producer. The maximum read index is described by the current
        // write index
        currentReadIndex        = m_readIndex;
        currentMaximumReadIndex = m_writeIndex;

        if (countToIndex(currentReadIndex) == 
            countToIndex(currentMaximumReadIndex))
        {
            // the queue is empty or
            // a producer thread has allocate space in the queue but is 
            // waiting to commit the data into it
            return false;
        }

        // retrieve the data from the queue
        a_data = m_theQueue[countToIndex(currentReadIndex)];

        // try to perfrom now the CAS operation on the read index. If we succeed
        // a_data already contains what m_readIndex pointed to before we 
        // increased it
        if (CAS(&m_readIndex, currentReadIndex, (currentReadIndex + 1)))
        {
            // got here. The value was retrieved from the queue. Note that the
            // data inside the m_queue array is not deleted nor reseted
#ifdef ARRAY_LOCK_FREE_Q_KEEP_REAL_SIZE
            AtomicSub(&m_count, 1);
#endif
            return true;
        }
        
        // it failed retrieving the element off the queue. Someone else must
        // have read the element stored at countToIndex(currentReadIndex)
        // before we could perform the CAS operation        

    } while(1); // keep looping to try again!

    // Something went wrong. it shouldn't be possible to reach here
    assert(0);

    // Add this return statement to avoid compiler warnings
    return false;
}

#endif // __ARRAY_LOCK_FREE_QUEUE_SINGLE_PRODUCER_IMPL_H__

