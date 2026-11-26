// Environment-variable controls shared by the demos.
//
// The demos take no command-line arguments: an interactive launch is always
// just the executable, and the headless/capture hooks (frame counts, timing
// CSVs, screenshot exports, automated camera paths, self-tests) ride
// DEMO_* environment variables instead. One shared parser keeps the
// conventions identical across demos: every helper names the offending
// variable in its error, unset variables keep the defaults, and values are
// range-checked exactly like the flags they replace were.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace demo::environment
{

// True when the variable is set to an affirmative value ("1", "true", "on",
// "yes" - case-insensitive). Any other value is an error naming the variable,
// so a typo cannot silently disable a requested mode.
bool readFlag(const char *name, bool &outValue, std::string &outError);

// Unsigned integer in [minimum, maximum]. outValue carries the caller's
// DEFAULT in and is only overwritten when the variable is set and valid, so
// callers seed it from their option's default before the call; the error
// names the variable, the raw value, and the accepted range.
bool readUint64(const char *name, std::uint64_t &outValue, std::uint64_t minimum, std::uint64_t maximum,
    std::string &outError);

// Finite double in [minimum, maximum]; outValue carries the default in and
// is only overwritten when the variable is set and valid (see readUint64).
bool readDouble(const char *name, double &outValue, double minimum, double maximum, std::string &outError);

// DEMO_SET: semicolon-separated "key=value" kernel configuration overrides
// (the same assignments the configuration layer merges after profiles, so a
// staged demo value still wins over the file and an explicit DEMO_SET entry
// wins over both).
bool readConfigurationOverrides(std::vector<std::string> &outOverrides, std::string &outError);

// DEMO_PROFILES: semicolon-separated configuration profile paths, resolved
// against the working directory like any path an operator types. The
// application's own [profile] forwarding reads the same list.
bool readProfilePaths(std::vector<std::string> &outPaths, std::string &outError);

// A unique screenshot file name for the interactive F12 export:
// "<prefix>_YYYYMMDD_HHMMSS_<index>.png" in the working directory, so
// repeated presses never overwrite each other and the files sort by time.
std::string timestampedScreenshotPath(const std::string &prefix, std::uint64_t index);

} // namespace demo::environment
