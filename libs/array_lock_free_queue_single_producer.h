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
/// @file array_lock_free_queue_single_producer.h
/// @brief Definition of a circular array based lock-free queue
///
/// WARNING: This queue is not thread safe when several threads try to insert 
///          elements into the queue. It is allowed to use as many consumers
///          as needed though.
///
/// @author Faustino Frechilla
/// @history
/// Ref  Who                 When         What
///      Faustino Frechilla  11-Jul-2010  Original development
/// @endhistory
/// 
// ============================================================================

#ifndef __ARRAY_LOCK_FREE_QUEUE_SINGLE_PRODUCER_H__
#define __ARRAY_LOCK_FREE_QUEUE_SINGLE_PRODUCER_H__

#include <stdint.h>     // uint32_t
#include "atomic_ops.h" // atomic operations wrappers

#define ARRAY_LOCK_FREE_Q_DEFAULT_SIZE 65536 // 2^16 = 65,536 elements by default

// define this variable if calls to "size" must return the real size of the 
// queue. If it is undefined  that function will try to take a snapshot of 
// the queue, but returned value might be bogus
#undef ARRAY_LOCK_FREE_Q_KEEP_REAL_SIZE
//#define ARRAY_LOCK_FREE_Q_KEEP_REAL_SIZE 1


/// @brief especialisation of the ArrayLockFreeQueue to be used when there is
///        only one producer thread
/// No allocation of extra memory for the nodes handling is needed
/// WARNING: This queue is not thread safe when several threads try to insert elements
/// into the queue
/// ELEM_T     represents the type of elementes pushed and popped from the queue
/// TOTAL_SIZE size of the queue. It should be a power of 2 to ensure 
///            indexes in the circular queue keep stable when the uint32_t 
///            variable that holds the current position rolls over from FFFFFFFF
///            to 0. For instance
///            2    -> 0x02 
///            4    -> 0x04
///            8    -> 0x08
///            16   -> 0x10
///            (...) 
///            1024 -> 0x400
///            2048 -> 0x800
///
///            if queue size is not defined as requested, let's say, for
///            instance 100, when current position is FFFFFFFF (4,294,967,295)
///            index in the circular array is 4,294,967,295 % 100 = 95. 
///            When that value is incremented it will be set to 0, that is the 
///            last 4 elements of the queue are not used when the counter rolls
///            over to 0
template <typename ELEM_T, uint32_t Q_SIZE = ARRAY_LOCK_FREE_Q_DEFAULT_SIZE>
class ArrayLockFreeQueueSingleProducer
{
public:
    /// @brief constructor of the class
    ArrayLockFreeQueueSingleProducer();
    virtual ~ArrayLockFreeQueueSingleProducer();

    /// @brief returns the current number of items in the queue
    /// It tries to take a snapshot of the size of the queue, but in busy environments
    /// this function might return bogus values. 
    ///
    /// If a reliable queue size must be kept you might want to have a look at 
    /// the preprocessor variable in this header file called 'ARRAY_LOCK_FREE_Q_KEEP_REAL_SIZE'
    /// it enables a reliable size though it hits overall performance of the queue 
    /// (when the reliable size variable is on it's got an impact of about 20% in time)
    uint32_t size();

    /// @brief push an element at the tail of the queue
    /// @param the element to insert in the queue
    /// Note that the element is not a pointer or a reference, so if you are using large data
    /// structures to be inserted in the queue you should think of instantiate the template
    /// of the queue as a pointer to that large structure
    /// @returns true if the element was inserted in the queue. False if the queue was full
    bool push(const ELEM_T &a_data);

    /// @brief pop the element at the head of the queue
    /// @param a reference where the element in the head of the queue will be saved to
    /// Note that the a_data parameter might contain rubbish if the function returns false
    /// @returns true if the element was successfully extracted from the queue. False if the queue was empty
    bool pop(ELEM_T &a_data);

private:
    /// @brief array to keep the elements
    ELEM_T m_theQueue[Q_SIZE];

#ifdef ARRAY_LOCK_FREE_Q_KEEP_REAL_SIZE
    /// @brief number of elements in the queue
    volatile uint32_t m_count;
#endif

    /// @brief where a new element will be inserted
    volatile uint32_t m_writeIndex;

    /// @brief where the next element where be extracted from
    volatile uint32_t m_readIndex;

    /// @brief calculate the index in the circular array that corresponds
    /// to a particular "count" value
    inline uint32_t countToIndex(uint32_t a_count);
};

// include the implementation file
#include "array_lock_free_queue_single_producer_impl.h"

#endif // __ARRAY_LOCK_FREE_QUEUE_SINGLE_PRODUCER_H__
