// sandbox_options.cpp - parsing for the physics sandbox demo's application
// arguments. Only the demo's own flags live here; engine options are parsed
// by parseApplicationCommandLine before this file sees anything.
#include "sandbox_options.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace demo::physics_sandbox
{
namespace
{

bool parseDouble(const std::string &text, double &outValue)
{
    try
    {
        std::size_t  consumed = 0;
        const double value    = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(value))
        {
            return false;
        }
        outValue = value;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool parseUnsigned(const std::string &text, std::uint64_t &outValue)
{
    try
    {
        std::size_t              consumed = 0;
        const unsigned long long value    = std::stoull(text, &consumed);
        if (consumed != text.size())
        {
            return false;
        }
        outValue = static_cast<std::uint64_t>(value);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

} // namespace

bool parseSandboxOptions(const std::vector<std::string> &arguments, SandboxOptions &options, std::string &outError)
{
    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        const std::string &argument = arguments[index];
        const auto         value
            = [&argument, &index, &arguments, &outError](double *number, std::uint64_t *integer) -> const std::string *
        {
            if (index + 1 >= arguments.size())
            {
                outError = "option '" + argument + "' expects a value";
                return nullptr;
            }
            const std::string &text = arguments[++index];
            if (number != nullptr && !parseDouble(text, *number))
            {
                outError = "option '" + argument + "' expects a number, got '" + text + "'";
                return nullptr;
            }
            if (integer != nullptr && !parseUnsigned(text, *integer))
            {
                outError = "option '" + argument + "' expects a non-negative integer, got '" + text + "'";
                return nullptr;
            }
            return &text;
        };

        if (argument == "--layout")
        {
            const std::string *text = value(nullptr, nullptr);
            if (text == nullptr)
            {
                return false;
            }
            if (*text != "showcase" && *text != "stacks" && *text != "cloth" && *text != "fields"
                && *text != "rope" && *text != "water" && *text != "fluid" && *text != "particles"
                && *text != "foliage" && *text != "character" && *text != "lod")
            {
                outError = "unknown layout '" + *text
                           + "' (showcase, stacks, cloth, fields, rope, water, fluid, particles, foliage, "
                             "character, lod)";
                return false;
            }
            options.sceneName = *text;
        }
        else if (argument == "--camera")
        {
            const std::string *text = value(nullptr, nullptr);
            if (text == nullptr)
            {
                return false;
            }
            if (*text != "orbit" && *text != "free" && *text != "fixed")
            {
                outError = "unknown camera mode '" + *text + "' (orbit, free, fixed)";
                return false;
            }
            options.cameraMode = *text;
        }
        else if (argument == "--fixed-hz")
        {
            double number = 0.0;
            if (value(&number, nullptr) == nullptr)
            {
                return false;
            }
            if (number < 15.0 || number > 1000.0)
            {
                outError = "--fixed-hz must be within [15, 1000]";
                return false;
            }
            options.fixedHz = number;
        }
        else if (argument == "--frames")
        {
            if (value(nullptr, &options.frames) == nullptr)
            {
                return false;
            }
        }
        else if (argument == "--warmup-seconds")
        {
            if (value(&options.warmupSeconds, nullptr) == nullptr)
            {
                return false;
            }
            if (options.warmupSeconds < 0.0 || options.warmupSeconds > 600.0)
            {
                outError = "--warmup-seconds must be within [0, 600]";
                return false;
            }
        }
        else if (argument == "--screenshot")
        {
            const std::string *text = value(nullptr, nullptr);
            if (text == nullptr)
            {
                return false;
            }
            options.screenshotPath = *text;
        }
        else if (argument == "--screenshot-frame")
        {
            if (value(nullptr, &options.screenshotFrame) == nullptr)
            {
                return false;
            }
        }
        else if (argument == "--frame-times")
        {
            const std::string *text = value(nullptr, nullptr);
            if (text == nullptr)
            {
                return false;
            }
            options.frameTimesPath = *text;
        }
        else if (argument == "--stats-every")
        {
            std::uint64_t integer = 0;
            if (value(nullptr, &integer) == nullptr)
            {
                return false;
            }
            options.statsEveryFrames = static_cast<std::uint32_t>(integer);
        }
        else if (argument == "--scripted")
        {
            options.scripted = true;
        }
        else if (argument == "--self-test")
        {
            options.selfTest = true;
        }
        else if (argument == "--pause")
        {
            options.pause = true;
        }
        else if (argument == "--dump-bodies")
        {
            options.dumpBodies = true;
        }
        else if (argument == "--no-articulation")
        {
            options.noArticulation = true;
        }
        else if (argument == "--no-cloth")
        {
            options.noCloth = true;
        }
        else
        {
            outError = "unknown option '" + argument + "'";
            return false;
        }
    }
    return true;
}

std::string sandboxOptionsUsage()
{
    std::ostringstream usage;
    usage << "\nphysics sandbox options:\n"
          << "  --layout <name>           showcase (default), stacks, cloth, fields, rope, water,\n"
          << "                            fluid, particles, foliage, character, or lod\n"
          << "  --camera <mode>           orbit (default), free, or fixed\n"
          << "  --fixed-hz <hz>           physics fixed step rate override [15, 1000]\n"
          << "  --frames <count>          render exactly count frames (0 = until window close)\n"
          << "  --warmup-seconds <t>      simulate t seconds before the first rendered frame\n"
          << "  --screenshot <path>       export the current frame as PPM\n"
          << "  --screenshot-frame <n>    0-based rendered frame to export (default last)\n"
          << "  --frame-times <path>      write per-frame timing CSV\n"
          << "  --stats-every <n>         print statistics every n frames (0 = never)\n"
          << "  --scripted                run the deterministic capture timeline\n"
          << "  --self-test               run determinism and frame-rate independence tests\n"
          << "  --pause                   start paused (Space toggles)\n"
          << "  --dump-bodies             print tracked body positions at stats intervals\n"
          << "  --no-articulation         skip the articulated figures\n"
          << "  --no-cloth                skip the cloth sheets\n";
    return usage.str();
}

} // namespace demo::physics_sandbox
