//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2004 Gregory Pakosz. Licensed under the WTFPLv2 License <http://www.wtfpl.net>.
//

// Borrowed and modified from https://github.com/gpakosz/PPK_ASSERT

#include "SpCore/Assert.h"

#include <CoreGlobals.h> // IsRunningCommandlet

#include "SpCore/Boost.h"
#include "SpCore/SuppressCompilerWarnings.h"
#include "SpCore/Windows.h"

SP_BEGIN_SUPPRESS_COMPILER_WARNINGS

// ------------------------------------------------------------------------------

// see README.md for usage instructions.
// (‑●‑●)> released under the WTFPL v2 license, by Gregory Pakosz (@gpakosz)

// ---- BEGIN SPEAR COMMENT ----
// #if defined(_WIN32)
// #define WIN32_LEAN_AND_MEAN
// #define _CRT_SECURE_NO_WARNINGS
// #include <windows.h>
// #endif
//
// #include <ppk_assert.h>
// ---- END SPEAR COMMENT ----

#include <cstdio>  // fprintf() and vsnprintf()
#include <cstring>
#include <cstdarg> // va_start() and va_end()
#include <cstdlib> // abort()

#if defined(__APPLE__)
#include <TargetConditionals.h>
#endif

#if defined(__ANDROID__) || defined(ANDROID)
#include <android/log.h>
#if !defined(PPK_ASSERT_LOG_TAG)
#define PPK_ASSERT_LOG_TAG "PPK_ASSERT"
#endif
#endif

//#define PPK_ASSERT_LOG_FILE "/tmp/assert.txt"
//#define PPK_ASSERT_LOG_FILE_TRUNCATE

// malloc and free are only used by AssertionException implemented in terms of
// short string optimization.
// However, no memory allocation happens if
// PPK_ASSERT_EXCEPTION_MESSAGE_BUFFER_SIZE == PPK_ASSERT_MESSAGE_BUFFER_SIZE
// which is the default.
#if !defined(PPK_ASSERT_MALLOC)
#define PPK_ASSERT_MALLOC(size) malloc(size)
#endif

#if !defined(PPK_ASSERT_FREE)
#define PPK_ASSERT_FREE(p) free(p)
#endif

#if !defined(PPK_ASSERT_MESSAGE_BUFFER_SIZE)
#  define PPK_ASSERT_MESSAGE_BUFFER_SIZE PPK_ASSERT_EXCEPTION_MESSAGE_BUFFER_SIZE
#endif

#if !defined(PPK_ASSERT_ABORT)
#define PPK_ASSERT_ABORT abort
#endif

namespace {

  namespace AssertLevel = ppk::assert::implementation::AssertLevel;
  namespace AssertAction = ppk::assert::implementation::AssertAction;

  typedef int (*printHandler)(FILE* out, int, const char* format, ...);

#if defined(PPK_ASSERT_LOG_FILE) && defined(PPK_ASSERT_LOG_FILE_TRUNCATE)
  struct LogFileTruncate
  {
    LogFileTruncate()
    {
      if (FILE* f = fopen(PPK_ASSERT_LOG_FILE, "w"))
        fclose(f);
    }
  };

  static LogFileTruncate truncate;
#endif

  int print(FILE* out, int level, const char* format, ...)
  {
    va_list args;
    int count;

    va_start(args, format);
    count = vfprintf(out, format, args);
    fflush(out);
    va_end(args);

#if defined(PPK_ASSERT_LOG_FILE)
    struct Local
    {
      static void log(const char* _format, va_list _args)
      {
        if (FILE* f = fopen(PPK_ASSERT_LOG_FILE, "a"))
        {
          vfprintf(f, _format, _args);
          fclose(f);
        }
      }
    };

    va_start(args, format);
    Local::log(format, args);
    va_end(args);
#endif

#if defined(_WIN32)
    char buffer[PPK_ASSERT_MESSAGE_BUFFER_SIZE];
    va_start(args, format);
    vsnprintf(buffer, PPK_ASSERT_MESSAGE_BUFFER_SIZE, format, args);
    ::OutputDebugStringA(buffer);
    va_end(args);
#endif

#if defined(__ANDROID__) || defined(ANDROID)
    int priority = ANDROID_LOG_VERBOSE;

    if (level >= AssertLevel::Debug)
      priority = ANDROID_LOG_DEBUG;
    else if (level >= AssertLevel::Warning)
      priority = ANDROID_LOG_WARN;
    else if (level >= AssertLevel::Error)
      priority = ANDROID_LOG_ERROR;
    else if (level >= AssertLevel::Fatal)
      priority = ANDROID_LOG_FATAL;

    va_start(args, format);
    __android_log_vprint(priority, PPK_ASSERT_LOG_TAG, format, args);
    va_start(args, format);
#else
    PPK_ASSERT_UNUSED(level);
#endif

    return count;
  }

  int formatLevel(int level, const char* expression, FILE* out, printHandler print)
  {
    const char* levelstr = 0;

    switch (level)
    {
      case AssertLevel::Debug:
        levelstr = "DEBUG";
        break;
      case AssertLevel::Warning:
        levelstr = "WARNING";
        break;
      case AssertLevel::Error:
        levelstr = "ERROR";
        break;
      case AssertLevel::Fatal:
        levelstr = "FATAL";
        break;

      default:
        break;
    }

    if (levelstr)
      return print(out, level, "Assertion '%s' failed (%s)\n", expression, levelstr);
    else
      return print(out, level, "Assertion '%s' failed (level = %d)\n", expression, level);
  }

  AssertAction::AssertAction PPK_ASSERT_CALL _defaultHandler( const char* file,
                                                              int line,
                                                              const char* function,
                                                              const char* expression,
                                                              int level,
                                                              const char* message)
  {
#if defined(_WIN32)
    if (::GetConsoleWindow() == NULL)
    {
      if (::AllocConsole())
      {
        (void)freopen("CONIN$", "r", stdin);
        (void)freopen("CONOUT$", "w", stdout);
        (void)freopen("CONOUT$", "w", stderr);

        SetFocus(::GetConsoleWindow());
      }
    }
#endif

    formatLevel(level, expression, stderr, reinterpret_cast<printHandler>(print));
    print(stderr, level, "  in file %s, line %d\n  function: %s\n", file, line, function);

    if (message)
      print(stderr, level, "  with message: %s\n\n", message);

    if (level < AssertLevel::Debug)
    {
      return AssertAction::None;
    }
    else if (AssertLevel::Debug <= level && level < AssertLevel::Error)
    {
#if (!TARGET_OS_IPHONE && !TARGET_IPHONE_SIMULATOR) && (!defined(__ANDROID__) && !defined(ANDROID)) || defined(PPK_ASSERT_DEFAULT_HANDLER_STDIN)

      // ---- BEGIN SPEAR MODIFICATION ----
      //
      // In practice, this for loop doesn't interact cleanly with Unreal applications, and leads to a flood
      // of output to the console in a variety of situations, so we disable.
      //
      // for (;;)
      // {
      //
      // If we're in the GUI editor, then we can't expect to get a user response via fgets. In this case,
      // break-then-throw if we're in a debugger, throw otherwise.

      #if WITH_EDITOR // defined in an auto-generated header
        if (!IsRunningCommandlet()) { // editor mode via GUI
          if (boost::debug::under_debugger()) {
            return AssertAction::BreakThenThrow;
          } else {
            return AssertAction::Throw;
          }
        }
      #endif

// #if defined(PPK_ASSERT_DISABLE_IGNORE_LINE)
//         fprintf(stderr, "Press (I)gnore / Ignore (A)ll / (D)ebug / A(b)ort: ");
// #else
//         fprintf(stderr, "Press (I)gnore / Ignore (F)orever / Ignore (A)ll / (D)ebug / A(b)ort: ");
// #endif

        // Adding an option to throw.

#if defined(PPK_ASSERT_DISABLE_IGNORE_LINE)
        fprintf(stderr, "Press (I)gnore / Ignore (A)ll / (D)ebug / Debug if available then (T)hrow / Th(r)ow / A(b)ort: ");
#else
        fprintf(stderr, "Press (I)gnore / Ignore (F)orever / Ignore (A)ll / (D)ebug / Debug if available then (T)hrow / Th(r)ow / A(b)ort: ");
#endif

        // ---- END SPEAR MODIFICATION ----

        fflush(stderr);

        char buffer[256];
        if (!fgets(buffer, sizeof(buffer), stdin))
        {
          clearerr(stdin);
          fprintf(stderr, "\n");
          fflush(stderr);

          // ---- BEGIN SPEAR MODIFICATION ----
          //
          // continue;
          //
          // If fgets returns null while waiting for input, then break-then-throw if we're in a debugger,
          // throw otherwise.

          if (boost::debug::under_debugger()) {
            return AssertAction::BreakThenThrow;
          } else {
            return AssertAction::Throw;
          }

          // ---- END SPEAR MODIFICATION ----
        }

        // we eventually skip the leading spaces but that's it
        char input[2] = {'b', 0};
        if (sscanf(buffer, " %1[a-zA-Z] ", input) != 1)

          // ---- BEGIN SPEAR MODIFICATION ----
          //
          // continue;
          //
          // If sscanf returns the wrong number of items, then break-then-throw if we're in a debugger, throw
          // otherwise.

          {
            if (boost::debug::under_debugger()) {
              return AssertAction::BreakThenThrow;
            } else {
              return AssertAction::Throw;
            }
          }

          // ---- END SPEAR MODIFICATION ----
 
        switch (*input)
        {
          case 'b':
          case 'B':
            return AssertAction::Abort;

          case 'd':
          case 'D':
            return AssertAction::Break;

          case 'i':
          case 'I':
            return AssertAction::Ignore;

#  if !defined(PPK_ASSERT_DISABLE_IGNORE_LINE)
          case 'f':
          case 'F':
            return AssertAction::IgnoreLine;
#  endif

          case 'a':
          case 'A':
            return AssertAction::IgnoreAll;

          // ---- BEGIN SPEAR MODIFICATION ----

          case 't':
          case 'T':
            if (boost::debug::under_debugger()) {
              return AssertAction::BreakThenThrow;
            } else {
              return AssertAction::Throw;
            }

          case 'r':
          case 'R':
            return AssertAction::Throw;

          // ---- END SPEAR MODIFICATION ----

          default:
            break;
        }

      // ---- BEGIN SPEAR MODIFICATION ----
      // }
      // ---- END SPEAR MODIFICATION ----

#else
      return AssertAction::Break;
#endif
    }
    else if (AssertLevel::Error <= level && level < AssertLevel::Fatal)
    {
      return AssertAction::Throw;
    }

    // ---- BEGIN SPEAR MODIFICATION ----

    fprintf(stderr, "Unrecognized input, choosing A(b)ort...\n");

    // ---- END SPEAR MODIFICATION ----

    return AssertAction::Abort;
  }

  void _throw(const char* file,
              int line,
              const char* function,
              const char* expression,
              const char* message)
  {
    using ppk::assert::implementation::throwException;
    throwException(ppk::assert::AssertionException(file, line, function, expression, message));
  }
}

namespace ppk {
namespace assert {

  AssertionException::AssertionException(const char* file,
                                         int line,
                                         const char* function,
                                         const char* expression,
                                         const char* message)
  : _file(file), _line(line), _function(function), _expression(expression), _heap(PPK_ASSERT_NULLPTR)
  {
    if (!message)
    {
      memset(_stack, 0, sizeof(char) * size);
      return;
    }

    size_t length = strlen(message);

    if (length < size) // message is short enough for the stack buffer
    {
      memcpy(_stack, message, sizeof(char) * length);
      memset(_stack + length, 0, sizeof(char) * (size - length)); // pad with 0
    }
    else // allocate storage on the heap
    {
      _heap = static_cast<char*>(PPK_ASSERT_MALLOC(sizeof(char) * (length + 1)));

      if (!_heap) // allocation failed
      {
        memcpy(_stack, message, sizeof(char) * (size - 1)); // stack fallback, truncate :/
        _stack[size - 1] = 0;
      }
      else
      {
        memcpy(_heap, message, sizeof(char) * length); // copy the message
        _heap[length] = 0;
        _stack[size - 1] = 1; // mark the stack
      }
    }
  }

  AssertionException::AssertionException(const AssertionException& rhs)
  : _file(rhs._file), _line(rhs._line), _function(rhs._function), _expression(rhs._expression)
  {
    const char* message = rhs.what();
    size_t length = strlen(message);

    if (length < size) // message is short enough for the stack buffer
    {
      memcpy(_stack, message, sizeof(char) * size); // pad with 0
    }
    else // allocate storage on the heap
    {
      _heap = static_cast<char*>(PPK_ASSERT_MALLOC(sizeof(char) * (length + 1)));

      if (!_heap) // allocation failed
      {
        memcpy(_stack, message, sizeof(char) * (size - 1)); // stack fallback, truncate :/
        _stack[size - 1] = 0;
      }
      else
      {
        memcpy(_heap, message, sizeof(char) * length); // copy the message
        _heap[length] = 0;
        _stack[size - 1] = 1; // mark the stack
      }
    }
  }

  AssertionException::~AssertionException() PPK_ASSERT_EXCEPTION_NO_THROW
  {
    if (_stack[size - 1])
      PPK_ASSERT_FREE(_heap);

    _heap = PPK_ASSERT_NULLPTR; // in case the exception object is destroyed twice
    _stack[size - 1] = 0;
  }

  AssertionException& AssertionException::operator = (const AssertionException& rhs)
  {
    if (&rhs == this)
      return *this;

    const char* message = rhs.what();
    size_t length = strlen(message);

    if (length < size) // message is short enough for the stack buffer
    {
      if (_stack[size - 1])
        PPK_ASSERT_FREE(_heap);

      memcpy(_stack, message, sizeof(char) * size);
    }
    else // allocate storage on the heap
    {
      if (_stack[size - 1])
      {
        size_t _length = strlen(_heap);

        if (length <= _length)
        {
          memcpy(_heap, message, sizeof(char) * _length); // copy the message, pad with 0
          return *this;
        }
        else
        {
          PPK_ASSERT_FREE(_heap);
        }
      }

      _heap = static_cast<char*>(PPK_ASSERT_MALLOC(sizeof(char) * (length + 1)));

      if (!_heap) // allocation failed
      {
        memcpy(_stack, message, sizeof(char) * (size - 1)); // stack fallback, truncate :/
        _stack[size - 1] = 0;
      }
      else
      {
        memcpy(_heap, message, sizeof(char) * length); // copy the message
        _heap[length] = 0;
        _stack[size - 1] = 1; // mark the stack
      }
    }

    _file = rhs._file;
    _line = rhs._line;
    _function = rhs._function;
    _expression = rhs._expression;

    return *this;
  }

  const char* AssertionException::what() const PPK_ASSERT_EXCEPTION_NO_THROW
  {
    return _stack[size - 1] ? _heap : _stack;
  }

namespace implementation {

  namespace {
    bool _ignoreAll = false;
  }

  void PPK_ASSERT_CALL ignoreAllAsserts(bool value)
  {
    _ignoreAll = value;
  }

  bool PPK_ASSERT_CALL ignoreAllAsserts()
  {
    return _ignoreAll;
  }

  namespace {
    AssertHandler _handler = _defaultHandler;
  }

  AssertHandler PPK_ASSERT_CALL setAssertHandler(AssertHandler handler)
  {
    AssertHandler previous = _handler;

    _handler = handler ? handler : _defaultHandler;

    return previous;
  }

  AssertAction::AssertAction PPK_ASSERT_CALL handleAssert(const char* file,
                                                          int line,
                                                          const char* function,
                                                          const char* expression,
                                                          int level,
                                                          bool* ignoreLine,
                                                          const char* message, ...)
  {
    char message_[PPK_ASSERT_MESSAGE_BUFFER_SIZE] = {0};
    const char* file_;

    if (message)
    {
      va_list args;
      va_start(args, message);
      vsnprintf(message_, PPK_ASSERT_MESSAGE_BUFFER_SIZE, message, args);
      va_end(args);

      message = message_;
    }

#if defined(_WIN32)
    file_ = strrchr(file, '\\');
#else
    file_ = strrchr(file, '/');
#endif // #if defined(_WIN32)

    file = file_ ? file_ + 1 : file;
    AssertAction::AssertAction action = _handler(file, line, function, expression, level, message);

    switch (action)
    {
      case AssertAction::Abort:
        PPK_ASSERT_ABORT();

#if !defined(PPK_ASSERT_DISABLE_IGNORE_LINE)
      case AssertAction::IgnoreLine:
        *ignoreLine = true;
        break;
#else
      PPK_ASSERT_UNUSED(ignoreLine);
#endif

      case AssertAction::IgnoreAll:
        ignoreAllAsserts(true);
        break;

      case AssertAction::Throw:
        _throw(file, line, function, expression, message);
        break;

      case AssertAction::Ignore:
      case AssertAction::Break:
      case AssertAction::None:
      default:
        return action;
    }

    return AssertAction::None;
  }

  // ---- BEGIN SPEAR MODIFICATION ----

  void PPK_ASSERT_CALL handleThrow(const char* file,
                                   int line,
                                   const char* function,
                                   const char* expression,
                                   const char* message, ...)
  {
    char message_[PPK_ASSERT_MESSAGE_BUFFER_SIZE] = {0};
    const char* file_;

    if (message)
    {
      va_list args;
      va_start(args, message);
      vsnprintf(message_, PPK_ASSERT_MESSAGE_BUFFER_SIZE, message, args);
      va_end(args);

      message = message_;
    }

#if defined(_WIN32)
    file_ = strrchr(file, '\\');
#else
    file_ = strrchr(file, '/');
#endif // #if defined(_WIN32)

    file = file_ ? file_ + 1 : file;

    _throw(file, line, function, expression, message);
  }

  // ---- END SPEAR MODIFICATION ----

} // namespace implementation
} // namespace assert
} // namespace ppk

// ------------------------------------------------------------------------------

SP_END_SUPPRESS_COMPILER_WARNINGS
