/**
 * @file grabcommon.h
 * @author Simone Comari
 * @date 14 Sep 2018
 * @brief This file collects common utilities which are not specifically related to any of GRAB
 * libraries.
 */

#ifndef GRABCOMMON_H
#define GRABCOMMON_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#define ANSI_COLOR_RED "\x1b[31m"   /**< ANSI _red_ codex for colorful printings. */
#define ANSI_COLOR_GREEN "\x1b[32m"   /**< ANSI _green_ codex for colorful printings. */
#define ANSI_COLOR_YELLOW "\x1b[33m"  /**< ANSI _yellow_ codex for colorful printings. */
#define ANSI_COLOR_BLUE "\x1b[34m"  /**< ANSI _blue_ codex for colorful printings. */
#define ANSI_COLOR_MAGENTA "\x1b[35m"  /**< ANSI _magenta_ codex for colorful printings. */
#define ANSI_COLOR_CYAN "\x1b[36m"  /**< ANSI _cyan_ codex for colorful printings. */
#define ANSI_COLOR_RESET "\x1b[0m"  /**< ANSI color reset codex after colorful printings. */

/**
 * @brief Handle an error message accorging to @c errno convention and display a message.
 * @note This function immediately terminates the process with an error.
 * @param[in] en Error number.
 * @param[in] msg Message to be displayed before error description.
 * @note For further details about error number convention, click
 * <a href="http://man7.org/linux/man-pages/man3/errno.3.html">here</a>.
 */
[[noreturn]] inline void HandleErrorEn(const int en, const char* msg)
{
  errno = en;
  perror(msg);
  exit(EXIT_FAILURE);
}

#endif // GRABCOMMON_H
