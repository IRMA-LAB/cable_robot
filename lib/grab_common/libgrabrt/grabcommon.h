/**
 * @file grabcommon.h
 * @author Simone Comari
 * @date 14 Sep 2018
 * @brief ...
 */

#ifndef GRABCOMMON_H
#define GRABCOMMON_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#define ANSI_COLOR_RED "\x1b[31m"   /**< ... */
#define ANSI_COLOR_GREEN "\x1b[32m"   /**< ... */
#define ANSI_COLOR_YELLOW "\x1b[33m"  /**< ...*/
#define ANSI_COLOR_BLUE "\x1b[34m"  /**< ...*/
#define ANSI_COLOR_MAGENTA "\x1b[35m"  /**< ...*/
#define ANSI_COLOR_CYAN "\x1b[36m"  /**<... */
#define ANSI_COLOR_RESET "\x1b[0m"  /**< ...*/

/**
 * @brief HandleErrorEn
 */
[[noreturn]] inline void HandleErrorEn(const int en, const char* msg)
{
  errno = en;
  perror(msg);
  exit(EXIT_FAILURE);
}

#endif // GRABCOMMON_H
