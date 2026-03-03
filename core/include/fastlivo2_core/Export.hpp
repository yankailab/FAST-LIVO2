#pragma once

#if defined(_WIN32)
  #if defined(FASTLIVO2_CORE_BUILD)
    #define FASTLIVO2_CORE_API __declspec(dllexport)
  #else
    #define FASTLIVO2_CORE_API __declspec(dllimport)
  #endif
#else
  #define FASTLIVO2_CORE_API __attribute__((visibility("default")))
#endif

