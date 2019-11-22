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
/// @file atomic_ops.h
/// @brief This file contains functions to wrap the built-in atomic operations 
///        defined for your compiler
///
/// @author Faustino Frechilla
/// @history
/// Ref  Who                 When         What
///      Faustino Frechilla  11-Jul-2010  Original development. GCC support
/// @endhistory
/// 
// ============================================================================

#ifndef __ATOMIC_OPS_H
#define __ATOMIC_OPS_H

#ifdef __GNUC__
// Atomic functions in GCC are present from version 4.1.0 on
// http://gcc.gnu.org/onlinedocs/gcc-4.1.0/gcc/Atomic-Builtins.html

// Test for GCC >= 4.1.0
#if (__GNUC__ < 4) || \
    ((__GNUC__ == 4) && ((__GNUC_MINOR__ < 1) || \
                        ((__GNUC_MINOR__     == 1) && \
                         (__GNUC_PATCHLEVEL__ < 0))) )
                        
#error Atomic built-in functions are only available in GCC in versions >= 4.1.0
#endif // end of check for GCC 4.1.0

/// @brief atomically adds a_count to the variable pointed by a_ptr
/// @return the value that had previously been in memory
#define AtomicAdd(a_ptr,a_count) __sync_fetch_and_add (a_ptr, a_count)

/// @brief atomically substracts a_count from the variable pointed by a_ptr
/// @return the value that had previously been in memory
#define AtomicSub(a_ptr,a_count) __sync_fetch_and_sub (a_ptr, a_count)

/// @brief Compare And Swap
///        If the current value of *a_ptr is a_oldVal, then write a_newVal into *a_ptr
/// @return true if the comparison is successful and a_newVal was written
#define CAS(a_ptr, a_oldVal, a_newVal) __sync_bool_compare_and_swap(a_ptr, a_oldVal, a_newVal)

/// @brief Compare And Swap
///        If the current value of *a_ptr is a_oldVal, then write a_newVal into *a_ptr
/// @return the contents of *a_ptr before the operation
#define CASVal(a_ptr, a_oldVal, a_newVal) __sync_val_compare_and_swap(a_ptr, a_oldVal, a_newVal)

#else
#error Atomic functions such as CAS or AtomicAdd are not defined for your compiler. Please add them in atomic_ops.h
#endif // __GNUC__


#endif // __ATOMIC_OPS_H
