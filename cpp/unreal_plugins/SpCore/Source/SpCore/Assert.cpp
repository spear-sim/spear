//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2004 Gregory Pakosz. Licensed under the WTFPLv2 License <http://www.wtfpl.net>.
//

// Borrowed and modified from https://github.com/gpakosz/PPK_ASSERT

#include "SpCore/Assert.h"

#include <mutex>  // std::lock_guard, std::recursive_mutex
#include <string> // std::string

#include <HAL/PlatformMisc.h>      // FPlatformMisc
#include <HAL/PlatformStackWalk.h> // FPlatformStackWalk

#include "SpCore/Boost.h"
#include "SpCore/Log.h"
#include "SpCore/SuppressCompilerWarnings.h"
#include "SpCore/Unreal.h"
#include "SpCore/Windows.h"

// ---- BEGIN SPEAR MODIFICATION ----

#if BOOST_OS_MACOS || BOOST_OS_LINUX
    #include <unistd.h> // fileno, isatty
#endif

// ---- END SPEAR MODIFICATION ----

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

    // ---- BEGIN SPEAR MODIFICATION ----

    // We call Log::logStringToUnreal(...) directly here, rather than SP_LOG_NO_PREFIX(...), because we
    // already wrote this same content to stdout via vfprintf(...) above; SP_LOG_NO_PREFIX(...) would write
    // it to stdout a second time in addition to routing it to UE_LOG. logStringToUnreal(...) is a no-op
    // unless we're in editor mode via GUI, so we don't need to duplicate that condition here ourselves.
    {
      char buffer[PPK_ASSERT_MESSAGE_BUFFER_SIZE];
      va_start(args, format);
      vsnprintf(buffer, PPK_ASSERT_MESSAGE_BUFFER_SIZE, format, args);
      Log::logStringToUnreal(std::string("ERROR: ") + buffer);
      va_end(args);
    }

    // ---- END SPEAR MODIFICATION ----

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

  // Forward declaration; see the definition below for details.
  void _printCallStack(int32 num_frames_to_skip);

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

    // ---- BEGIN SPEAR MODIFICATION ----

    // Hold this lock across our entire report (header + call stack) so it can never be torn by an
    // interleaved write from another thread, e.g., GLog's dedicated background thread flushing an unrelated
    // message via StdOutputDevice::Serialize(...) at the same time (see Log::getStdoutMutex()'s comment for
    // the full explanation). We deliberately release it before the interactive prompt below, since that
    // prompt can block on keyboard input for an arbitrary amount of time, and we don't want to stall every
    // other thread's logging for as long as we happen to be sitting at the prompt.
    {
      std::lock_guard<std::recursive_mutex> lock(Log::getStdoutMutex());

      formatLevel(level, expression, stdout, reinterpret_cast<printHandler>(print));
      print(stdout, level, "  in file %s, line %d\n  function: %s\n", file, line, function);

      if (message)
        print(stdout, level, "  with message: %s\n\n", message);

      // Print the call stack here, before the interactive prompt below (which blocks on keyboard input),
      // so we always have a record of where the assert failed even if we end up sitting at the prompt for
      // a while. This is now the only place we print the call stack, regardless of which AssertAction we
      // ultimately resolve to. See _printCallStack(...) below for how num_frames_to_skip was chosen.
      int32 num_frames_to_skip = 3;
      _printCallStack(num_frames_to_skip);
    }

    //
    // On macOS and Linux, a double-clicked executable has no controlling terminal attached to stdin, so
    // there is no way to show the interactive prompt below or read a response to it. In this situation, we
    // decide automatically based on whether a debugger is available and whether we're currently executing
    // inside a region (e.g., EngineService::executeFuncInTryCatch(...)) that knows how to gracefully
    // recover from a thrown exception:
    //
    //     debugger    | throw allowed      |            | BreakThenThrow | inspect then throw and let the exception be handled
    //     debugger    | throw not allowed  |            | Break          | inspect then continue since throwing an exception here would be uncaught
    //     no debugger | throw allowed      |            | Throw          | throw and let the exception be handled
    //     no debugger | throw not allowed  | editor     | Crash          | let the UE crash handler try to display the call stack
    //     no debugger | throw not allowed  | no editor  | Exit           | exit to avoid leaving a zombie process that can take a long time to clean up
    //
    // On Windows we don't need this special case, because we already forced a console into existence above
    // if one didn't already exist.
    //

    AssertAction::AssertAction default_assert_action = AssertAction::None;
    if (boost::debug::under_debugger()) {
      if (AssertsAreAllowedToThrowScope::insideAssertsAreAllowedToThrowScope()) {
        default_assert_action = AssertAction::BreakThenThrow;
      } else {
        default_assert_action = AssertAction::Break;
      }
    } else {
      if (AssertsAreAllowedToThrowScope::insideAssertsAreAllowedToThrowScope()) {
        default_assert_action = AssertAction::Throw;
      } else {
        #if WITH_EDITOR
          default_assert_action = AssertAction::Crash;
        #else
          default_assert_action = AssertAction::Exit;
        #endif
      }
    }

    // ---- END SPEAR MODIFICATION ----

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

      #if BOOST_OS_MACOS || BOOST_OS_LINUX
        if (!isatty(fileno(stdin))) {
          return default_assert_action;
        }
      #endif

// #if defined(PPK_ASSERT_DISABLE_IGNORE_LINE)
//         fprintf(stderr, "Press (I)gnore / Ignore (A)ll / (D)ebug / A(b)ort: ");
// #else
//         fprintf(stderr, "Press (I)gnore / Ignore (F)orever / Ignore (A)ll / (D)ebug / A(b)ort: ");
// #endif

        // Adding multiple new options.

#if defined(PPK_ASSERT_DISABLE_IGNORE_LINE)
        fprintf(stdout, "Press (I)gnore / Ignore (A)ll / (D)ebug / Debug then (T)hrow / Th(r)ow / A(b)ort / (C)rash / E(x)it: ");
#else
        fprintf(stdout, "Press (I)gnore / Ignore (F)orever / Ignore (A)ll / (D)ebug / Debug then (T)hrow / Th(r)ow / A(b)ort / (C)rash / E(x)it: ");
#endif

        // ---- END SPEAR MODIFICATION ----

        fflush(stdout);

        char buffer[256];
        if (!fgets(buffer, sizeof(buffer), stdin))
        {
          clearerr(stdin);
          fprintf(stdout, "\n");
          fflush(stdout);

          // ---- BEGIN SPEAR MODIFICATION ----

          // continue;

          // If fgets returns null while waiting for input, then return the default assert action.

          return default_assert_action;

          // ---- END SPEAR MODIFICATION ----
        }

        // we eventually skip the leading spaces but that's it
        char input[2] = {'b', 0};
        if (sscanf(buffer, " %1[a-zA-Z] ", input) != 1)

          // ---- BEGIN SPEAR MODIFICATION ----

          // continue;

          // If sscanf returns the wrong number of items, then return the default assert action.

          return default_assert_action;

          // ---- END SPEAR MODIFICATION ----
 
        switch (*input)
        {
          case 'b':
          case 'B':
            return AssertAction::Abort;

          // ---- BEGIN SPEAR MODIFICATION ----

          case 'c':
          case 'C':
            return AssertAction::Crash;

          case 'd':
          case 'D':
            if (boost::debug::under_debugger()) {
              return AssertAction::Break;
            } else {
              return AssertAction::Exit;
            }

          // ---- END SPEAR MODIFICATION ----

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

          case 'x':
          case 'X':
            return AssertAction::Exit;

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

    // If no other assert action applies, then return the default assert action.

    return default_assert_action;

    // ---- END SPEAR MODIFICATION ----
  }

  // ---- BEGIN SPEAR MODIFICATION ----

  // Prints the current call stack. Called once, from _defaultHandler(...) above, immediately after the
  // assertion header is printed and before the interactive prompt (if any) blocks on keyboard input, so we
  // always have a record of where an assert failed regardless of which AssertAction we end up resolving to,
  // and regardless of whether a debugger is attached to show us the stack another way. num_frames_to_skip
  // is the number of frames to skip starting from (and including) this function's own frame; handleAssert(...)
  // calls _handler(...), which is _defaultHandler(...), which calls this function, so a skip count of 3
  // reaches the original SP_ASSERT call site.
  void _printCallStack(int32 num_frames_to_skip)
  {
    const int32 max_call_stack_chars = 65535;
    ANSICHAR call_stack[max_call_stack_chars] = {0};
    FPlatformStackWalk::StackWalkAndDump(call_stack, max_call_stack_chars, num_frames_to_skip);
    SP_LOG_NO_PREFIX("Call stack:");
    SP_LOG_NO_PREFIX(std::string(call_stack));
  }

  // Returns a human-readable description of a single call stack frame (e.g., "0x0b7f9868
  // SpearSim!SpCore::initializeSharedMemory() [UnknownFile]"), so we can pass something more useful than a
  // hardcoded string to FPlatformMisc::RequestExit(...)'s CallSite parameter. See _printCallStack(...) above
  // for how to choose num_frames_to_skip.
  std::string _getCallSiteString(int32 num_frames_to_skip)
  {
    const int32 max_depth = 16;
    SP_ASSERT(num_frames_to_skip < max_depth);

    uint64 back_trace[max_depth] = {0};
    uint32 num_frames_captured = FPlatformStackWalk::CaptureStackBackTrace(back_trace, num_frames_to_skip + 1);

    if (num_frames_captured <= static_cast<uint32>(num_frames_to_skip)) {
      return "";
    }

    const int32 max_buffer_chars = 8192;
    ANSICHAR buffer[max_buffer_chars] = {0};
    int32 current_call_depth = 0;
    FPlatformStackWalk::ProgramCounterToHumanReadableString(current_call_depth, back_trace[num_frames_to_skip], buffer, max_buffer_chars);
    return std::string(buffer);
  }

  // ---- END SPEAR MODIFICATION ----

  void _throw(const char* file,
              int line,
              const char* function,
              const char* expression,
              const char* message)
  {
    using ppk::assert::implementation::throwException;
    throwException(ppk::assert::AssertionException(file, line, function, expression, message));
  }

  // ---- BEGIN SPEAR MODIFICATION ----

  void _exit(const char* file,
             int line,
             const char* function,
             const char* expression,
             const char* message)
  {
    SP_LOG_NO_PREFIX("ERROR: Assertion '", expression, "' failed (", file, ":", line, ", ", function, ")");
    if (message) {
        SP_LOG_NO_PREFIX("    with message: ", message);
    }
    int32 num_frames_to_skip = 3;
    std::string call_site_string = _getCallSiteString(num_frames_to_skip);
    bool force = true;
    FPlatformMisc::RequestExit(force, Unreal::toTCharPtr(call_site_string));
  }

  void _crash(const char* file,
              int line,
              const char* function,
              const char* expression,
              const char* message)
  {
    SP_LOG_NO_PREFIX("ERROR: Assertion '", expression, "' failed (", file, ":", line, ", ", function, ")");
    if (message) {
        SP_LOG_NO_PREFIX("    with message: ", message);
    }

    // Deliberately trigger a hardware fault so Unreal's own crash-handling pipeline (e.g., the editor's
    // built-in crash handler) engages, in case it can produce a more complete call stack than the one
    // printed earlier in _defaultHandler(...).
    uint32 exception_code = 1;
    FPlatformMisc::RaiseException(exception_code);
  }

  // ---- END SPEAR MODIFICATION ----
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

      // ---- BEGIN SPEAR MODIFICATION ----

      case AssertAction::Exit:
        _exit(file, line, function, expression, message);
        break;

      case AssertAction::Crash:
        _crash(file, line, function, expression, message);
        break;

      // ---- END SPEAR MODIFICATION ----

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

  void PPK_ASSERT_CALL handleExit(const char* file,
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

    _exit(file, line, function, expression, message);
  }

  // ---- END SPEAR MODIFICATION ----

} // namespace implementation
} // namespace assert
} // namespace ppk

// ------------------------------------------------------------------------------

SP_END_SUPPRESS_COMPILER_WARNINGS

// ---- BEGIN SPEAR MODIFICATION ----

AssertsAreAllowedToThrowScope::AssertsAreAllowedToThrowScope()
{
    depth_++;
}

AssertsAreAllowedToThrowScope::~AssertsAreAllowedToThrowScope()
{
    depth_--;
}

bool AssertsAreAllowedToThrowScope::insideAssertsAreAllowedToThrowScope()
{
    return depth_ > 0;
}

// ---- END SPEAR MODIFICATION ----
