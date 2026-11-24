// main.cpp - the planet generation demo application.
//
// A standalone harness for the planet module: it creates one procedural
// planet through the "planet" capability, streams its LOD chunk meshes into
// the renderer every frame, and exposes screenshot exports and determinism
// self-tests so generation can be validated without any manual inspection.
//
// Views (deterministic camera placement from the planet's own sampling):
//   --view globe    whole planet from space: continents, oceans, ice caps.
//   --view orbit    mid-altitude sweep over the limbs and ranges.
//   --view coast    standing on a beach looking along the shoreline.
//   --view cliffs   terraced mountain country with overhanging cliff faces.
//   --view surface  rolling green hills under the sun.
//   --view walk     first person on the terrain: the planet module's
//                   SurfaceWalker carries the camera (WASD + mouse + Space +
//                   Shift in a window; --orbit auto-walks deterministically).
//
// Useful flags: --seed N / --radius M override the planet description,
// --frames N renders exactly N frames (deterministic; --warmup W prepends W
// build-only frames), --screenshot exports the rendered frame as PPM,
// --self-test verifies chunk generation is deterministic and independent of
// chunk build order and audits the walk controller, --stats-every prints
// planet statistics cadence, --caves enables the cave worm carving,
// --chunk-borders highlights chunk borders on the surface.

#include <engine/application/application.hpp>
#include <engine/application/frame_pacer.hpp>
#include <engine/camera/capabilities.hpp>
#include <engine/kernel/executable_directory.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/planet/capabilities.hpp>
#include <engine/planet/meshing.hpp>
#include <engine/planet/planet.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include <engine/window/capabilities.hpp>

#include "planet_camera_input.hpp"
#include "planet_chunk_renderer.hpp"
#include "planet_scatter.hpp"
#include "planet_demo_support.hpp"
#include "demo_harness.hpp"
#include "planet_walk_controller.hpp"
#include <engine/application/application_host.hpp>
#include <engine/application/command_line.hpp>
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
using namespace engine;
using namespace demo::planet_demo;
using demo::harness::FrameTimingRecorder;
namespace window = engine::window;
namespace renderer = engine::renderer;
namespace planet = engine::planet;

} // namespace

int main(int argc, char **argv)
{
    std::cout << std::unitbuf;

    engine::ApplicationCommandLineOptions commandLine;
    std::string                           commandLineError;
    if (!engine::parseApplicationCommandLine(argc, argv, commandLine, commandLineError))
    {
        std::cerr << "demo_planet: " << commandLineError << '\n'
                  << engine::applicationCommandLineUsage(argv[0]);
        return 2;
    }
    if (commandLine.helpRequested)
    {
        std::cout << engine::applicationCommandLineUsage(argv[0]) << planetOptionsUsage();
        return 0;
    }

    PlanetDemoOptions options;
    if (!parsePlanetOptions(commandLine.applicationArguments, options, commandLineError))
    {
        std::cerr << "demo_planet: " << commandLineError << "\n";
        return 2;
    }

    engine::ApplicationHostOptions hostOptions
        = engine::makeApplicationHostOptions(commandLine, {"window", "renderer", "camera", "pbr", "planet"});
    if (options.caves)
    {
        // The demo stages the toggle as a configuration override so the
        // planet module reads it during construct (the capability creates
        // planets from the configured template; there is no per-planet
        // post-creation knob for the cave field).
        hostOptions.configurationOverrides.push_back("planet.caves_enabled=true");
    }
    if (options.chunkBorders)
    {
        // Same staging as --caves: the overlay is baked into the chunk's
        // vertex colors at build time, so it rides the configured template.
        hostOptions.configurationOverrides.push_back("planet.debug_chunk_borders=true");
    }
    engine::ApplicationHost host;
    std::string             startupReport;
    if (commandLine.validateOnly)
    {
        const bool valid = host.validate(hostOptions, startupReport);
        std::cout << startupReport;
        std::cout << (valid ? "validate: ok\n" : "validate: failed\n");
        return valid ? 0 : 1;
    }
    if (!host.start(hostOptions, startupReport))
    {
        std::cerr << startupReport << "demo_planet: the engine failed to start\n";
        return 1;
    }
    if (!startupReport.empty())
    {
        std::cout << startupReport;
    }

    engine::ParticipantContext &context = host.context();
    const std::int64_t maxFps = context.configurationIntegerInSection("engine", "max_fps").value_or(0);

    window::WindowCapability   &windowCapability  = context.acquireCapability<window::WindowCapability>();
    renderer::RenderCapability &renderCapability  = context.acquireCapability<renderer::RenderCapability>();
    camera::CameraCapability   &cameraCapability  = context.acquireCapability<camera::CameraCapability>();
    auto *planetCapability = context.tryAcquireCapability<planet::PlanetCapability>();
    if (planetCapability == nullptr || planetCapability->abi != planet::planetCapabilityAbi)
    {
        std::cerr << "demo_planet: the planet capability is unavailable or its ABI differs; this demo is pointless "
                     "without it\n";
        host.stop();
        return 1;
    }
    const pbr::PbrCapability *pbrCapability = context.tryAcquireCapability<pbr::PbrCapability>();
    pbr::PbrEnvironment       environment{};
    if (pbrCapability != nullptr && pbrCapability->copyEnvironment(pbrCapability->self, &environment, nullptr))
    {
        std::cout << "demo_planet: pbr sun " << (environment.sun.enabled ? "enabled" : "disabled") << " (intensity "
                  << environment.sun.intensity << ")\n";
    }

    // ---- planet creation ---------------------------------------------------------
    planet::PlanetDescription description{};
    description.seed         = options.seed;
    description.radiusMeters = options.radius;
    const planet::PlanetHandle planetHandle = planetCapability->createPlanet(planetCapability->self, &description, nullptr);
    if (planetHandle == planet::invalidPlanet)
    {
        std::cerr << "demo_planet: the planet module rejected the planet description\n";
        host.stop();
        return 1;
    }
    planet::PlanetGenerationParams params{};
    planetCapability->copyGenerationParams(planetCapability->self, planetHandle, &params);

    // Sun direction for the whole demo: the view pickers (ground views
    // prefer lit slopes), the backdrop layers and the cloud-shadow check
    // below all key off the pbr sun.
    const planet::Vec3 sunDirection = environment.sun.enabled
        ? planet::Vec3{static_cast<double>(environment.sun.direction[0]),
            static_cast<double>(environment.sun.direction[1]),
            static_cast<double>(environment.sun.direction[2])}
        : planet::Vec3{0.0, 0.0, 0.0};

    std::cout << "demo_planet: planet seed " << params.seed << ", radius " << params.radiusMeters << " m, octree depth "
              << params.derivedMaxDepth << ", leaf voxel " << params.derivedVoxelMeters << " m\n";

    // Cloud-shadow sun consistency: the planet module casts cloud shadows
    // from its configured direction; the scene is LIT by the pbr sun. When
    // the two disagree the shadows fall from the wrong side - visible as
    // banks whose shade slides the wrong way. Both are configuration, so
    // the check is a startup warning instead of a hard failure.
    if (environment.sun.enabled && params.cloudShadowStrength > 0.0 && params.cloudsEnabled)
    {
        const planet::Vec3 shadowSun{params.cloudShadowSunDirection[0], params.cloudShadowSunDirection[1],
            params.cloudShadowSunDirection[2]};
        const planet::Vec3 litSun = planet::normalized(sunDirection);
        const double     agreement = planet::dot(shadowSun, litSun);
        if (agreement < 0.98)
        {
            std::cout << "demo_planet: warning - [planet] cloud_shadow_sun_direction disagrees with the [pbr] "
                         "sun_direction (cosine " << agreement << "); cloud shadows will not match the lighting\n";
        }
    }

    // ---- self-test short-circuit ----------------------------------------------------
    if (options.selfTest)
    {
        const SelfTestResult test = runSelfTest(*planetCapability, planetCapability->self, options);
        std::cout << test.report;
        planetCapability->destroyPlanet(planetCapability->self, planetHandle);
        host.stop();
        return test.pass ? 0 : 1;
    }

    // ---- benchmark short-circuit -----------------------------------------------------
    if (options.benchmark)
    {
        const int benchStatus = runBenchmark(params);
        planetCapability->destroyPlanet(planetCapability->self, planetHandle);
        host.stop();
        return benchStatus;
    }

    // ---- biome census short-circuit ---------------------------------------------------
    if (options.biomeCensusSamples != 0)
    {
        std::cout << "demo_planet: biome census (seed " << params.seed << ", climate field over the globe)\n";
        runBiomeCensus(*planetCapability, planetHandle, params, options.biomeCensusSamples);
        planetCapability->destroyPlanet(planetCapability->self, planetHandle);
        host.stop();
        return 0;
    }

    // ---- camera placement per view ----------------------------------------------------
    const glm::dvec3 planetCenter(params.centerMeters[0], params.centerMeters[1], params.centerMeters[2]);
    const double     planetRadius = params.radiusMeters;
    const bool       walkView     = options.viewName == "walk";

    glm::dvec3 eye      = planetCenter + glm::dvec3(0.0, 0.0, planetRadius * 3.0);
    glm::dvec3 target   = planetCenter;
    double     nearZ    = planetRadius * 0.01;
    double     farZ     = planetRadius * 12.0;

    // The walk controller owns the density-field walk surface; it is only
    // built for the walk view (one extra seed schedule, no worker threads).
    std::unique_ptr<PlanetWalkController> walkController;

    // Terrain windows shared by the surface view placements (they pin the
    // biome/climate range a view's spot may sit in).
    const auto elevationWindow = [](double low, double high)
    {
        return [low, high](double elevation, engine::planet::Biome, double, const planet::PlanetGenerationParams &)
        { return elevation >= low && elevation <= high; };
    };
    // Warm-land windows: the biome check alone is not enough - the biome
    // colors BLEND into snow below the climate snow line (and toward the
    // freeze line), so a spot the enum calls "taiga" can still render
    // half-white. Require strictly warm air and a spot clearly below the
    // blended snow band so showcase views land on green/sandy terrain.
    const auto warmLandWindow = [](double low, double high)
    {
        return [low, high](double elevation, engine::planet::Biome biome, double temperature01,
            const planet::PlanetGenerationParams &params)
        {
            if (elevation < low || elevation > high)
            {
                return false;
            }
            if (biome == planet::Biome::Snow || biome == planet::Biome::Tundra || temperature01 <= 0.45)
            {
                return false;
            }
            const double snowLine = params.snowLineMeters * (0.25 + 0.75 * temperature01);
            const double snowBand = params.snowLineMeters * params.elevationBlendFraction;
            return elevation < snowLine - snowBand;
        };
    };
    // Mountain-foothill window for the cliffs view: terraced mountain
    // country is cold (the lapse rate takes 0.2-0.8 temperature01 off by
    // 400-900 m of altitude), so a warm-land window zeroes nearly every
    // mountain candidate - only tropical ranges survive, and those rarely
    // carry terraced flanks at showcase scale. Taiga/tundra air is the
    // honest climate on terraced massifs; the picker's own gates add the
    // terraced-mountain requirement (it needs the relief sample, which this
    // predicate signature has not).
    const auto mountainFoothillWindow = [](double low, double high)
    {
        return [low, high](double elevation, engine::planet::Biome biome, double,
            const planet::PlanetGenerationParams &)
        {
            return elevation >= low && elevation <= high
                && biome != planet::Biome::Snow && biome != planet::Biome::Beach;
        };
    };
    const auto warmGreenLandWindow = [warmLandWindow](double low, double high)
    {
        auto warm = warmLandWindow(low, high);
        return [warm](double elevation, engine::planet::Biome biome, double temperature01,
            const planet::PlanetGenerationParams &params)
        {
            // TropicalRainforest is excluded: its canopy palette is the
            // darkest row (linear albedo ~0.09-0.13) and a front-lit
            // frame over it still photographs as a murky near-black
            // sheet under the demo's linear exposure.
            const bool green = biome == planet::Biome::Grassland
                || biome == planet::Biome::TemperateForest || biome == planet::Biome::SeasonalForest
                || biome == planet::Biome::TemperateRainforest
                || biome == planet::Biome::Taiga;
            return warm(elevation, biome, temperature01, params) && green;
        };
    };

    if (options.viewName == "globe")
    {
        // The eye sits ~72 degrees OFF the sun: the terminator crosses the
        // disc, relief reads through cast shadows, and the atmosphere limb
        // glows on one side - a view straight down the sun direction lights
        // every slope equally and the planet photographs flat. The eye stays
        // lifted above the equator so the north polar cap and its sea ice
        // stay in frame.
        const glm::dvec3 sun = environment.sun.enabled
            ? glm::normalize(glm::dvec3(sunDirection.x, sunDirection.y, sunDirection.z))
            : glm::normalize(glm::dvec3(0.38, 0.30, 0.87));
        glm::dvec3       side = glm::cross(sun, glm::dvec3(0.0, 1.0, 0.0));
        if (glm::length(side) < 1.0e-6)
        {
            side = glm::cross(sun, glm::dvec3(1.0, 0.0, 0.0));
        }
        side            = glm::normalize(side);
        const glm::dvec3 direction = glm::normalize(sun * 0.30 + side * 0.72 + glm::dvec3(0.0, 0.35, 0.0));
        eye                        = planetCenter + direction * planetRadius * 3.1;
        target                     = planetCenter;
    }
    else if (options.viewName == "orbit")
    {
        // Mid-altitude view: the planet fills most of the frame while the
        // limb and its atmosphere ring stay visible - the scale between the
        // globe and the surface where range structure (or its absence) shows
        // most clearly. The eye sits ~63 degrees off the sun so the ranges
        // model through shading and the terminator touches the limb.
        const glm::dvec3 sun = environment.sun.enabled
            ? glm::normalize(glm::dvec3(sunDirection.x, sunDirection.y, sunDirection.z))
            : glm::normalize(glm::dvec3(0.38, 0.30, 0.87));
        glm::dvec3       side = glm::cross(sun, glm::dvec3(0.0, 1.0, 0.0));
        if (glm::length(side) < 1.0e-6)
        {
            side = glm::cross(sun, glm::dvec3(1.0, 0.0, 0.0));
        }
        side            = glm::normalize(side);
        const glm::dvec3 direction = glm::normalize(sun * 0.45 + side * 0.80 + glm::dvec3(0.0, 0.30, 0.0));
        eye                        = planetCenter + direction * planetRadius * 2.2;
        target                     = planetCenter;
    }
    else if (walkView)
    {
        // First-person walk: spawn on green inland terrain (the same warm
        // green window the surface view uses - it guarantees walkable,
        // non-snowy, non-desert country), then let the walker stand on the
        // terrain top at 1.7 m eye height. The probe height keeps the
        // viewpoint's sight caps calibrated while the walker itself decides
        // the exact standing height from the field.
        Viewpoint viewpoint;
        bool found = pickViewpoint(*planetCapability, planetHandle, warmGreenLandWindow(2.0, 200.0), ProbePolicy::Inland,
            60.0, sunDirection, viewpoint);
        if (!found)
        {
            found = pickViewpoint(*planetCapability, planetHandle, elevationWindow(1.0, 800.0), ProbePolicy::Inland,
                60.0, sunDirection, viewpoint);
        }
        if (!found)
        {
            std::cerr << "demo_planet: no usable walk spawn found for view 'walk'\n";
            planetCapability->destroyPlanet(planetCapability->self, planetHandle);
            host.stop();
            return 1;
        }
        walkController = std::make_unique<PlanetWalkController>(params, planetCenter);
        walkController->spawn(viewpoint.direction);
        walkController->setAutoWalk(options.orbit);
        eye    = walkController->eye();
        target = walkController->target();
        // Walking sees metres, not kilometres: a 0.1 m near plane keeps the
        // ground under the feet from clipping while the far plane still
        // reaches the horizon.
        nearZ = 0.1;
        farZ  = planetRadius * 2.0;
        std::cout << "demo_planet: walk spawn - eye (" << eye.x << ", " << eye.y << ", " << eye.z << ") radius "
                  << glm::length(eye - planetCenter) << "\n";
    }
    else
    {
        // Surface views: find a deterministic spot matching the view's
        // terrain predicate, stand eyeHeight above the ground, look along the
        // horizon toward the view's focus.
        Viewpoint viewpoint;
        bool found = false;
        // View placement: biome-aware, climate-aware and sightline-checked so
        // the view shows landscape rather than the nearest slope wall.

        // Biome-pinned sample views (--biome <name>): the spot must sit in
        // exactly that climate zone, so a capture photographs one biome's
        // ground, flora and water. Water biomes stand over open sea (the
        // seabed camera would render the underwater sheet edge-on); the
        // beach keeps its shoreline framing; steep rock faces use the
        // mountain probe rings.
        engine::planet::Biome pinnedBiome = engine::planet::Biome::Ocean;
        const bool biomePinned = !options.biomeName.empty() && biomeFromName(options.biomeName, pinnedBiome);
        bool overWater = false;
        if (biomePinned)
        {
            overWater = pinnedBiome == engine::planet::Biome::DeepOcean
                || pinnedBiome == engine::planet::Biome::Ocean
                || pinnedBiome == engine::planet::Biome::ShallowOcean;
            const bool forestPin = pinnedBiome == engine::planet::Biome::TemperateForest
                || pinnedBiome == engine::planet::Biome::SeasonalForest
                || pinnedBiome == engine::planet::Biome::TropicalRainforest
                || pinnedBiome == engine::planet::Biome::TemperateRainforest
                || pinnedBiome == engine::planet::Biome::Taiga;
            const double deepFloor = pinnedBiome == engine::planet::Biome::DeepOcean ? -8000.0
                : (pinnedBiome == engine::planet::Biome::Ocean ? -1500.0 : -150.0);
            // Forest pins stay off the coastal flats (their planed-off sand
            // shelves photograph as a beach with stray trees instead of the
            // biome's own country).
            const double low = overWater ? deepFloor : (forestPin ? 80.0 : 0.5);
            const double high = overWater ? (pinnedBiome == engine::planet::Biome::ShallowOcean ? -2.0 : 0.0)
                : 3000.0;
            const auto biomeWindow = [pinnedBiome, overWater, low, high](double elevation, engine::planet::Biome biome,
                double temperature01, const planet::PlanetGenerationParams &params)
            {
                if (biome != pinnedBiome || elevation < low || elevation > high)
                {
                    return false;
                }
                if (overWater)
                {
                    // A water-biome sample photographs the biome's WATER: the
                    // sea freezes where the SEA-LEVEL temperature drops under
                    // the freeze band, and a pin that lands there photographs
                    // an ice sheet instead of deep-ocean water. The climate
                    // sample's temperature01 is the SEABED's lapse-adjusted
                    // value (depth warms it by |elevation|/lapse), so back out
                    // the sea-level figure with the same lapse and require it
                    // above the ice transition, plus a margin so the sample
                    // sits in open water, not on the noisy ice edge.
                    const double seaLevelTemperature = temperature01
                        + elevation / std::max(1.0, params.temperatureLapseMeters);
                    const double iceFree = params.seaIceTemperature
                        + std::max(1.0e-4, params.seaIceTransition) + 0.02;
                    return seaLevelTemperature > iceFree;
                }
                if (pinnedBiome == engine::planet::Biome::Rock)
                {
                    // Rock's classifier identity is slope-only, but the
                    // RENDERED look keys on the snow blend: a cliff face
                    // above the climate's snow line photographs as a
                    // snowfield with rock peeking out, not as the bare-rock
                    // biome. Keep rock pins in warm cliff country - below
                    // the blended snow band AND with the spot's own air warm
                    // enough that the mid-field mesa rims (several hundred
                    // meters above the eye) stay under the blend too, where
                    // the same snowLine(t) rule the classifier and mesher use
                    // leaves the strata reading as rock, not frost.
                    const double snowLine = params.snowLineMeters * (0.25 + 0.75 * temperature01);
                    const double snowBand = params.snowLineMeters * params.elevationBlendFraction;
                    if (elevation >= snowLine - snowBand || temperature01 < 0.30)
                    {
                        return false;
                    }
                }
                return true;
            };
            // All land pins use the Pinned policy (the Mountain spot gate
            // additionally demands mountainFactor - a bare-rock face need not
            // be a massif, and the neighborhood matching already prefers
            // deep rock country).
            const ProbePolicy policy = pinnedBiome == engine::planet::Biome::Beach && !overWater
                ? ProbePolicy::Coastline
                : ProbePolicy::Pinned;
            // Water rings: the pinned water biome's own depth band qualifies
            // everywhere (the probe loop accepts open-sea hops for water pins;
            // the land+water composition gates stand down - open water IS the
            // sample); land rings: the biome's own country qualifies everywhere.
            const engine::planet::Biome *neighborhoodBiome = &pinnedBiome;
            // The picker's probe height must match the camera's eye height
            // below (140 m for land pins, 25 m over water) so the sight-fan
            // wall caps clear exactly the walls the rendered frame will see.
            found = pickViewpoint(*planetCapability, planetHandle, biomeWindow, overWater ? ProbePolicy::Coastline : policy,
                overWater ? 25.0 : 140.0, sunDirection, viewpoint, neighborhoodBiome);
            if (!found)
            {
                // Pathological seeds (or water floors deeper than the gate's
                // elevation window): retry with the elevation band widened -
                // water pins keep the ice-free gate (the relaxed retry must
                // not land the camera back on a frozen sea).
                const auto relaxed = [pinnedBiome, overWater](double elevation, engine::planet::Biome biome,
                    double temperature01, const planet::PlanetGenerationParams &params)
                {
                    if (biome != pinnedBiome)
                    {
                        return false;
                    }
                    if (overWater)
                    {
                        const double seaLevelTemperature = temperature01
                            + elevation / std::max(1.0, params.temperatureLapseMeters);
                        const double iceFree = params.seaIceTemperature
                            + std::max(1.0e-4, params.seaIceTransition) + 0.02;
                        return seaLevelTemperature > iceFree;
                    }
                    return true;
                };
                found = pickViewpoint(*planetCapability, planetHandle, relaxed,
                    overWater ? ProbePolicy::Coastline : policy, overWater ? 25.0 : 140.0, sunDirection, viewpoint,
                    neighborhoodBiome);
            }
            if (!found && pinnedBiome == engine::planet::Biome::Desert)
            {
                // Arid-belt substitution: on seeds whose dry belts never
                // bottom out past the desert row, the honest arid sample is
                // the HOT SAVANNA (the classifier's next-drier row), not a
                // mislabeled green spot. The substitution is logged and the
                // sample sheet documents it.
                static const engine::planet::Biome kHotSavanna = engine::planet::Biome::Savanna;
                const auto hotArid = [](double, engine::planet::Biome biome, double temperature01,
                    const planet::PlanetGenerationParams &)
                { return biome == engine::planet::Biome::Savanna && temperature01 > 0.55; };
                found = pickViewpoint(*planetCapability, planetHandle, hotArid, ProbePolicy::Pinned, 140.0,
                    sunDirection, viewpoint, &kHotSavanna);
                if (found)
                {
                    std::cout << "demo_planet: biome 'desert' not present on this seed; capturing the hot arid savanna belt instead (the classifier's next-drier row)\n";
                }
            }
            if (!found && pinnedBiome == engine::planet::Biome::TemperateRainforest)
            {
                // Wet-temperate substitution: this seed's moisture field
                // never crosses the rainforest row inside the temperate
                // bands (the census reports the zone absent), so the honest
                // wet-forest sample is the TEMPERATE FOREST - the same
                // temperature band's next-wetter realized row. The
                // substitution is logged and the sample sheet documents it.
                static const engine::planet::Biome kTemperateWoodland = engine::planet::Biome::TemperateForest;
                const auto wetTemperate = [](double, engine::planet::Biome biome, double temperature01,
                    const planet::PlanetGenerationParams &)
                { return biome == engine::planet::Biome::TemperateForest && temperature01 > 0.30 && temperature01 < 0.50; };
                found = pickViewpoint(*planetCapability, planetHandle, wetTemperate, ProbePolicy::Pinned, 140.0,
                    sunDirection, viewpoint, &kTemperateWoodland, 1);
                if (found)
                {
                    std::cout << "demo_planet: biome 'temperate_rainforest' not present on this seed; capturing its temperate-forest neighbor instead (the same band's next-wetter realized row, a different pocket than the temperate_forest sample)\n";
                }
            }
            if (found)
            {
                std::cout << "demo_planet: biome-pinned view '" << options.biomeName << "' - elevation "
                          << (viewpoint.surfaceRadius - params.derivedSeaRadiusMeters) << " m\n";
            }
        }
        else if (options.viewName == "coast")
        {
            // A warm beach seen from just above the shoreline; the sea may
            // (should) enter the frame. The window reaches 45 m: the coastal
            // flattening damps most shoreline terrain toward the waterline,
            // so a 2-18 m window nearly only ever matched tiny islets whose
            // land hides under the camera.
            found = pickViewpoint(*planetCapability, planetHandle, warmLandWindow(2.0, 45.0), ProbePolicy::Coastline, 120.0,
                sunDirection, viewpoint);
        }
        else if (options.viewName == "cliffs")
        {
            // Below the snow blend, at any elevation where terraced cliff
            // faces and overhangs live (the snow blend rides the climate
            // line, so the window adapts to temperature). Higher country than
            // the other views: rock faces and strata need real relief. The
            // picker adds the terraced-country gate (cliff mask at the spot).
            found = pickViewpoint(*planetCapability, planetHandle, mountainFoothillWindow(150.0, 1200.0), ProbePolicy::Mountain, 140.0,
                sunDirection, viewpoint);
        }
        else // surface
        {
            // Rolling green hills: the SPOT itself must be a genuinely green
            // biome (shrubland is excluded - its pale khaki albedo photographs
            // as washed-out empty country), above the coastal-flatten band
            // (its planed-off terrain photographs as an empty sheet) and
            // below the snow blend.
            found = pickViewpoint(*planetCapability, planetHandle, warmGreenLandWindow(60.0, 200.0), ProbePolicy::Inland, 140.0,
                sunDirection, viewpoint);
        }
        if (!found)
        {
            // Very flat or very mountainous seeds may lack a match; fall back
            // to any land-ish point rather than failing the demo.
            found = pickViewpoint(*planetCapability, planetHandle, elevationWindow(1.0, 2000.0), ProbePolicy::Coastline, 110.0,
                sunDirection, viewpoint);
        }
        if (!found)
        {
            // Last resort: the coastline gate (land AND water inside the
            // sight fan) can fail on pathological seeds; an inland land view
            // always beats an error exit.
            found = pickViewpoint(*planetCapability, planetHandle, elevationWindow(1.0, 2000.0), ProbePolicy::Inland, 140.0,
                sunDirection, viewpoint);
        }
        if (!found)
        {
            std::cerr << "demo_planet: no usable viewpoint found for view '" << options.viewName << "'\n";
            planetCapability->destroyPlanet(planetCapability->self, planetHandle);
            host.stop();
            return 1;
        }

        const glm::dvec3 direction(viewpoint.direction.x, viewpoint.direction.y, viewpoint.direction.z);
        // Water biome samples stand over the SEA SURFACE (the spot radius is
        // the seabed below it; standing there renders the water sheet from
        // beneath, not the biome's water).
        const glm::dvec3 position = planetCenter
            + direction * (overWater ? params.derivedSeaRadiusMeters : viewpoint.surfaceRadius);
        {
            // Degenerate or non-finite tangent: fall back to the world-up
            // cross product below.
            const planet::Vec3 &stored = viewpoint.lookTangent;
            const bool usable = std::isfinite(stored.x) && std::isfinite(stored.y) && std::isfinite(stored.z)
                && planet::length(stored) > 0.5;
            if (!usable)
            {
                viewpoint.hasLookTangent = false;
            }
        }
        // Tangent frame at the spot: coast views get the waterward direction
        // the picker chose; everything else uses the world-up cross tangent.
        const glm::dvec3 up       = direction;
        glm::dvec3       tangent  = viewpoint.hasLookTangent
            ? glm::dvec3(viewpoint.lookTangent.x, viewpoint.lookTangent.y, viewpoint.lookTangent.z)
            : glm::normalize(glm::cross(up, glm::dvec3(0.0, 1.0, 0.0)));
        if (viewpoint.hasLookTangent)
        {
            tangent = glm::normalize(tangent);
        }
        else if (glm::length(tangent) < 0.5)
        {
            tangent = glm::normalize(glm::cross(up, glm::dvec3(1.0, 0.0, 0.0)));
        }

        // Camera eye heights MATCH the picker's altitudes: the sightline wall
        // caps are calibrated at exactly these heights, and the old mismatch
        // (camera at 220/420 m after the caps cleared 110/140 m) let walls
        // the caps never saw fill the frame.
        const double eyeHeight = overWater ? 25.0
            : (options.viewName == "cliffs" ? 140.0 : (options.viewName == "surface" ? 140.0 : 120.0));
        eye                    = position + up * eyeHeight;
        // The cliffs view keeps its subject HALF the look-ahead of the other
        // views: a terrace riser is a few tens of meters tall, and the
        // angular LOD rule only resolves it (and the strata bands riding
        // it) when the wall sits well inside a few hundred meters - at the
        // other views' distance the wall's chunks mesh at voxels coarse
        // enough to smooth the riser into a ramp.
        const double lookAheadDistance = overWater ? 220.0
            : (options.viewName == "cliffs" ? 380.0 : (options.viewName == "surface" ? 420.0 : 320.0));
        // Exact pitch aim below the local horizontal - no terrain
        // re-projection (when the ground ahead rises, a re-projected target
        // snaps onto the hillside wall and fills the frame with slope) and
        // no level tangent aim (it put the horizon mid-frame, the shoreline
        // out of it, and half the frame black sky). Per-view pitch centers
        // the subject: the coast shoreline sits ~15 deg below a 120 m eye at
        // 320 m; surface ground at 420 m sits ~15.5 deg below a 140 m eye;
        // the cliffs view aims near level AT the terraced wall the picker
        // located (the wall rises ahead and fills the center frame).
        const double aimPitchRadians = overWater ? 0.14
            : (options.viewName == "cliffs" ? 0.12
                : (options.viewName == "surface" ? 0.27 : 0.26)); // 8 / 6.9 / 15.5 / 14.9 deg
        target = eye + (tangent * std::cos(aimPitchRadians) - up * std::sin(aimPitchRadians)) * lookAheadDistance;
        // Ground views sit close to the terrain: a near plane scaled from
        // the eye height (55-105 m) clipped everything inside it. 15 m keeps
        // near clipping out of the frame while leaving depth precision for
        // the 120 km far plane.
        nearZ    = 15.0;
        farZ     = planetRadius * 2.0;
        // Open-framing guarantee: whatever the ground ahead does, the view
        // pitch stays in a band that keeps the horizon in frame (deep enough
        // to keep landscape in the lower half, shallow enough to keep sky
        // above it). Cliff views may pitch deeper - the risers below the eye
        // are the showcase. (The exact-pitch aim above already lands inside
        // this band; the clamp is a safety net.)
        const double maxPitchRadians = options.viewName == "cliffs" ? 0.52359877559829882 : 0.38397243543875248; // 30 deg / 22 deg
        clampCameraPitch(eye, up, target, 0.08726646259971647, maxPitchRadians); // >= 5 deg down
        std::cout << "demo_planet: view pose - eye (" << eye.x << ", " << eye.y << ", " << eye.z << ") radius "
                  << glm::length(eye - planetCenter) << ", target (" << target.x << ", " << target.y << ", "
                  << target.z << ")\n";
    }

    // ---- window / renderer / camera ------------------------------------------------
    window::WindowDescription  windowDescription{};
    window::WindowError        windowError = window::WindowError::None;
    const window::WindowHandle windowHandle
        = windowCapability.createWindow(windowCapability.self, windowDescription, &windowError);
    if (windowHandle == window::invalidWindow)
    {
        std::cerr << "demo_planet: the window module could not create a window\n";
        host.stop();
        return 1;
    }
    renderer::RenderError          renderError = renderer::RenderError::None;
    const renderer::RendererHandle rendererHandle
        = renderCapability.createRenderer(renderCapability.self, windowHandle, &renderError);
    if (rendererHandle == renderer::invalidRenderer)
    {
        std::cerr << "demo_planet: the renderer module could not create a renderer\n";
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }
    const std::string windowBackendName = windowCapability.activeBackendName(windowCapability.self);
    std::cout << "demo_planet: window backend '" << windowBackendName << "', renderer backend '"
              << renderCapability.activeBackendName(renderCapability.self, rendererHandle) << "'\n";
    windowCapability.pollEvents(windowCapability.self);

    const bool windowedBackend = windowBackendName != std::string("headless");
    if (windowedBackend)
    {
        std::cout << "demo_planet: controls - mouse looks, WASD moves, Space jumps (walk) or rises, Ctrl descends,\n"
                     "      Shift sprints, arrows look, Left Alt toggles the cursor, Escape quits\n";
        if (options.orbit)
        {
            std::cout << "demo_planet: --orbit drives the camera automatically; disable it for manual movement\n";
        }
    }

    std::uint32_t framebufferWidth  = 0;
    std::uint32_t framebufferHeight = 0;
    windowCapability.getFramebufferSize(windowCapability.self, windowHandle, &framebufferWidth, &framebufferHeight);
    if (framebufferWidth == 0 || framebufferHeight == 0)
    {
        framebufferWidth  = 960;
        framebufferHeight = 540;
    }
    renderCapability.setViewport(renderCapability.self, rendererHandle, framebufferWidth, framebufferHeight);

    const bool macroView = options.viewName == "globe" || options.viewName == "orbit";
    glm::dvec3 initialCameraUp = macroView ? glm::dvec3(0.0, 1.0, 0.0)
                                          : glm::normalize(eye - planetCenter);
    if (macroView
        && glm::abs(glm::dot(glm::normalize(eye - planetCenter), glm::dvec3(0.0, 1.0, 0.0))) > 0.99)
    {
        // Avoid a singular look-at basis when the globe view starts near a pole.
        initialCameraUp = glm::normalize(glm::dvec3(0.05, 0.9987, 0.0));
    }

    // Camera movement scales with the view: close terrain uses walking/flying
    // speeds, while globe/orbit views need larger world-unit steps. The walk
    // view leaves the speed untouched - the walker owns movement, the camera
    // only carries the pose.
    const float cameraMoveSpeed = macroView
        ? std::clamp(static_cast<float>(planetRadius * (options.viewName == "globe" ? 0.05 : 0.025)), 250.0f, 50000.0f)
        : std::clamp(static_cast<float>(planetRadius * 0.001), 12.0f, 300.0f);

    // Field of view per view family (walk leans wide; globe frames the whole
    // planet; surface views lean wide for ground context). Hoisted so the
    // texture gate below and the camera share one source.
    const float cameraFovYDegrees = walkView ? 70.0f
        : (macroView ? (options.viewName == "orbit" ? 55.0f : 42.0f) : 65.0f);

    const camera::CameraHandle cameraHandle = [&]()
    {
        camera::CameraParameters cameraParameters{};
        cameraParameters.position[0] = static_cast<float>(eye.x);
        cameraParameters.position[1] = static_cast<float>(eye.y);
        cameraParameters.position[2] = static_cast<float>(eye.z);
        cameraParameters.hasPosition  = true;
        cameraParameters.target[0]    = static_cast<float>(target.x);
        cameraParameters.target[1]    = static_cast<float>(target.y);
        cameraParameters.target[2]    = static_cast<float>(target.z);
        cameraParameters.hasTarget    = true;
        cameraParameters.up[0]        = static_cast<float>(initialCameraUp.x);
        cameraParameters.up[1]        = static_cast<float>(initialCameraUp.y);
        cameraParameters.up[2]        = static_cast<float>(initialCameraUp.z);
        cameraParameters.hasUp        = true;
        cameraParameters.fovYDegrees  = cameraFovYDegrees;
        cameraParameters.nearZ        = static_cast<float>(nearZ);
        cameraParameters.farZ         = static_cast<float>(farZ);
        cameraParameters.moveSpeed    = cameraMoveSpeed;
        cameraParameters.lookSpeed    = 90.0f;
        cameraParameters.mouseSensitivity = 0.11f;
        cameraParameters.pitchLimitDegrees = 89.0f;
        camera::CameraError cameraError = camera::CameraError::None;
        return cameraCapability.createCamera(cameraCapability.self, &cameraParameters, &cameraError);
    }();
    if (cameraHandle == camera::invalidCamera)
    {
        std::cerr << "demo_planet: the camera module could not create a camera\n";
        renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
        windowCapability.destroyWindow(windowCapability.self, windowHandle);
        host.stop();
        return 1;
    }

    cameraCapability.setControlMode(cameraCapability.self, cameraHandle,
        walkView ? camera::CameraControlMode::Fixed : camera::CameraControlMode::Fly);
    PlanetCameraInput cameraInput(windowCapability, windowHandle, windowedBackend, !options.orbit);

    // ---- run mode -------------------------------------------------------------------
    const bool          runUntilClosed = options.frames == 0 && windowedBackend;
    const std::uint64_t totalFrames    = options.frames != 0 ? options.frames : 90;
    // Warmup frames stream chunk builds without rendering; they EXTEND the
    // run (frames are always rendered exactly as requested - warmup no longer
    // consumes the render budget). Warmup is a BUDGET, not an exact count:
    // once the visible set is fully built (and the pool is idle) the remaining
    // warmup frames are dropped, so a warmup value just needs to be large
    // enough for the view's chunk population on the current machine.
    std::uint64_t warmupRemaining = options.warmupFrames;
    // --settle-seconds window: while it is open the warmup budget is not
    // consumed (see the warmup block in the main loop). Ground views want
    // thousands of chunks that deliver at the build pool's own rate, so a
    // frame budget large enough for them depends on the machine's build
    // throughput - the seconds window expresses "wait until the visible set
    // is complete, but no longer than this" directly.
    const std::chrono::steady_clock::time_point settleStart = std::chrono::steady_clock::now();
    bool settleWindowOpen = options.settleSeconds > 0.0;
    // Completion must HOLD before warmup ends: the build pool drains in waves
    // (an octree pass discovers the next refinement ring only after the
    // current one finishes), so a single idle snapshot can catch the trough
    // between waves and end warmup with later rings still unbuilt - the
    // rendered frames would then draw coarse parent chunks (and re-stream)
    // where fine children were still in flight. Warmup ends only after this
    // many consecutive fully-built iterations.
    constexpr std::uint64_t settleHoldIterations = 90;
    std::uint64_t           settleHoldProgress   = 0;
    const double        frameDelta     = 1.0 / 30.0;
    if (!options.screenshotFrameSet)
    {
        // Without --screenshot-frame the capture lands on the last rendered
        // frame (0 remains a valid explicit index through the boolean).
        options.screenshotFrame = totalFrames - 1;
    }

    std::cout << "demo_planet: view '" << options.viewName << "', " << totalFrames << " frames ("
              << options.warmupFrames << " warmup";
    if (settleWindowOpen)
    {
        std::cout << " + settle window up to " << options.settleSeconds << "s";
    }
    std::cout << ")\n";
    // A --screenshot-frame beyond the run's frame count can never capture;
    // say so upfront (the lumen demo reports the same condition) instead of
    // finishing silently without the requested file.
    if (!options.screenshotPath.empty() && options.screenshotFrame >= totalFrames)
    {
        std::cout << "demo_planet: warning - --screenshot-frame " << options.screenshotFrame
                  << " is beyond this run of " << totalFrames << " frames; no screenshot will be taken\n";
    }
    // Warmup exhaustion record: a budget that runs out before the visible
    // set is complete means the first rendered frames (and any screenshot)
    // still contain streaming gaps - the operator should know the capture is
    // early rather than trust it blindly. The statistics are snapshotted at
    // the moment the budget ends; the run's final counts can only be more
    // complete (streaming continues through the rendered frames), so a
    // warning based on them would report a finished set and mislead.
    bool                       warmupExhausted      = false;
    planet::PlanetStatistics warmupExhaustionStats{};

    // GPU chunk residency and draw-command storage are isolated from the
    // application loop; this keeps planet streaming and camera control from
    // accumulating in one demo-level coordinator.
    PlanetChunkRenderer chunkRenderer(*planetCapability, planetHandle, renderCapability, rendererHandle, params);
    // Surface scatter: the ground-level real-world elements (vegetation,
    // surface rocks, bird flocks, breaking-wave foam, valley mist). The
    // system scatters deterministically against the planet module's density
    // rules; the chunk renderer composes its draw commands per frame.
    PlanetScatterSystem scatterSystem(*planetCapability, planetHandle, params);
    chunkRenderer.setSurfaceScatter(&scatterSystem);
    // Terrain surface textures (CC0 scans under assets/textures; see that
    // directory's README): the ground group's detail map, the rock group's
    // detail map and its normal map. The renderer multiplies them with the
    // vertex colors in linear space; a missing file leaves that group
    // untextured (the vertex colors stand alone) with one warning, so the
    // demo still runs from a checkout without the asset directory.
    //
    // Each group's multiply only applies to chunks within the gate distance:
    // one texture tile must still span at least --texture-tile-pixels screen
    // pixels. The CPU sampler filters through a full trilinear mip chain, so
    // a tile minified to a handful of pixels still contributes its macro
    // variation instead of shimmering. The gate follows from the viewport
    // height and the view's field of view (the tile's projected pixel span
    // grows linearly with height and shrinks with distance and zoom).
    // The default budget keeps the multiply where a tile still photographs as
    // DETAIL and cuts it where the scan's motif has minified into a repeating
    // stamp: past a few dozen pixels per tile the eye stops reading the tile's
    // content as grain and starts matching the tile lattice itself, and the
    // multiply then reads as wallpaper over the terrain (the vertex colors'
    // own large-scale mottle carries those distances instead, and the
    // per-chunk gate plus the mip chain fade the boundary so no ring shows
    // where the multiply stops). Larger budgets re-extend the texture for
    // deliberately tiled close-ups; 0 disables it outright (a zero gate
    // distance keeps every chunk past the gate, and the divide below would
    // otherwise turn the zero into an infinite gate - the exact opposite of
    // disabling).
    const double textureMinTilePixels = options.textureTilePixels != 0u
        ? static_cast<double>(options.textureTilePixels) : std::numeric_limits<double>::infinity();
    const double viewportHeightMeters = static_cast<double>(
        context.configurationIntegerInSection("window", "height").value_or(540));
    const double tanHalfFovY = std::tan(glm::radians(static_cast<double>(cameraFovYDegrees)) * 0.5);
    const double groundTextureGateMeters = params.terrainTextureTileMetersGround * viewportHeightMeters
        / (2.0 * tanHalfFovY * textureMinTilePixels);
    const double rockTextureGateMeters = params.terrainTextureTileMetersRock * viewportHeightMeters
        / (2.0 * tanHalfFovY * textureMinTilePixels);
    {
        const std::filesystem::path textureDirectory
            = engine::executableDirectory() / "assets" / "textures";
        const auto loadTexture = [&renderCapability, rendererHandle, &textureDirectory](const char *fileName) -> renderer::TextureHandle
        {
            const std::filesystem::path path = textureDirectory / fileName;
            renderer::RenderError error = renderer::RenderError::None;
            const renderer::TextureHandle handle
                = renderCapability.createTextureFromFile(renderCapability.self, rendererHandle, path.string().c_str(), &error);
            if (handle == renderer::invalidTexture)
            {
                std::cerr << "demo_planet: terrain texture '" << path.string()
                          << "' could not be loaded; the surface group it belongs to draws untextured\n";
            }
            return handle;
        };
        // Per-variant mean compensation, read back from the renderer instead
        // of hardcoding per-asset constants: the capability computes each
        // texture's mean over its decoded pixels at creation (see
        // RenderCapability::textureMean), and the chunk renderer divides the
        // gated material's base color by the SELECTED variant's mean, keeping
        // every variant's multiply albedo-preserving (see setTerrainTextures).
        const auto loadVariant = [&loadTexture, &renderCapability, rendererHandle](const char *detailFile,
            const char *normalFile) -> demo::planet_demo::TerrainTextureVariant
        {
            demo::planet_demo::TerrainTextureVariant variant;
            variant.detail = loadTexture(detailFile);
            if (normalFile != nullptr)
            {
                variant.normal = loadTexture(normalFile);
            }
            renderer::RenderError error = renderer::RenderError::None;
            if (variant.detail != renderer::invalidTexture
                && renderCapability.textureMean != nullptr)
            {
                renderCapability.textureMean(renderCapability.self, rendererHandle, variant.detail,
                    variant.meanCompensation, &error);
                // The mean compensates the multiply: divide the material's
                // base color by the scan's mean (per channel).
                for (int channel = 0; channel < 3; ++channel)
                {
                    variant.meanCompensation[channel]
                        = 1.0f / std::max(1.0e-4f, variant.meanCompensation[channel]);
                }
            }
            return variant;
        };
        // Each surface group carries a SET of scanned variants: the chunk
        // renderer picks one per chunk (deterministically from the chunk
        // address), so neighboring chunks photograph different ground and
        // rock scans and no scan repeats across the whole visible disk (see
        // setTerrainTextures). All scans come from Poly Haven under CC0
        // (see assets/textures/README.md).
        std::vector<demo::planet_demo::TerrainTextureVariant> groundVariants;
        groundVariants.push_back(loadVariant("ground_grey_diff_1k.jpg", nullptr));
        groundVariants.push_back(loadVariant("aerial_ground_rock_diff_1k.jpg", nullptr));
        groundVariants.push_back(loadVariant("dry_ground_rocks_diff_1k.jpg", nullptr));
        std::vector<demo::planet_demo::TerrainTextureVariant> rockVariants;
        rockVariants.push_back(loadVariant("cliff_side_diff_1k.jpg", "cliff_side_nor_gl_1k.jpg"));
        rockVariants.push_back(loadVariant("rocky_terrain_02_diff_1k.jpg", "rocky_terrain_02_nor_gl_1k.jpg"));
        rockVariants.push_back(loadVariant("rock_face_diff_1k.jpg", "rock_face_nor_gl_1k.jpg"));
        chunkRenderer.setTerrainTextures(groundVariants, rockVariants,
            groundTextureGateMeters, rockTextureGateMeters);
    }
    // The world-anchored cloud shell bakes the pbr environment's lighting
    // into its vertex colors and draws unlit (see BackdropLighting); the
    // values below are the same snapshot the renderer shades every lit draw
    // with.
    chunkRenderer.setBackdropLighting({glm::dvec3(environment.sun.direction[0], environment.sun.direction[1],
        environment.sun.direction[2]),
        glm::dvec3(environment.sun.color[0], environment.sun.color[1], environment.sun.color[2])
            * static_cast<double>(environment.sun.intensity) / glm::pi<double>(),
        glm::dvec3(environment.ambient.skyColor[0], environment.ambient.skyColor[1], environment.ambient.skyColor[2])
            * static_cast<double>(environment.ambient.intensity),
        glm::dvec3(environment.ambient.groundColor[0], environment.ambient.groundColor[1],
            environment.ambient.groundColor[2])
            * static_cast<double>(environment.ambient.intensity),
        environment.sun.enabled});
    // Backdrop shells upload once per run; the frame loop only places and
    // tints them. Draw order lives in rebuildDrawCommands (opaque scene
    // first, clouds depth-tested against it, backdrop shells last).
    chunkRenderer.ensureAtmosphereShell();
    chunkRenderer.ensureSkyDome();
    chunkRenderer.ensureSkyCloudLayer(eye - planetCenter,
        environment.sun.enabled ? glm::dvec3(sunDirection.x, sunDirection.y, sunDirection.z) : glm::dvec3(0.0));
    chunkRenderer.ensureCloudLayer();
    chunkRenderer.ensureStarDome();
    chunkRenderer.ensureSunDisc();
    chunkRenderer.ensureMoon();
    chunkRenderer.ensureAurora(
        environment.sun.enabled ? glm::dvec3(sunDirection.x, sunDirection.y, sunDirection.z) : glm::dvec3(0.0));
    chunkRenderer.setCloudLayerVisible(macroView);
    chunkRenderer.setSkyCloudLayerVisible(!macroView);
    scatterSystem.initialize(renderCapability, rendererHandle, engine::executableDirectory() / "assets" / "textures");

    // ---- frame loop -------------------------------------------------------------------
    std::uint64_t framesRendered = 0;
    FrameTimingRecorder frameTiming;
    if (!options.frameTimesPath.empty())
    {
        // Every completed frame rewrites the CSV atomically, so a run that is
        // interrupted mid-measurement (process kill, machine shutdown) still
        // keeps the timings of the frames it managed to render.
        frameTiming.setAutoFlushPath(options.frameTimesPath);
    }
    bool          screenshotTaken = false;
    double        orbitAngle      = 0.0;
    // Simulation clock for the orbital model (moon_orbital_period_seconds):
    // capture runs drive it by the fixed frame delta, so the companion's
    // position is a pure function of the frame index there.
    double        simSeconds      = 0.0;
    std::array<window::WindowEvent, 64> eventBuffer;
    auto lastWallClock = std::chrono::steady_clock::now();

    const bool presentedPacing = renderCapability.waitsForPresentation != nullptr
        && renderCapability.waitsForPresentation(renderCapability.self, rendererHandle);
    const bool pacerSuppressedByVsync = presentedPacing && maxFps > 0;
    const double effectiveTargetHz = pacerSuppressedByVsync ? 0.0 : static_cast<double>(std::max<std::int64_t>(maxFps, 0));
    const std::uint32_t pacerSpinWindowMicroseconds = effectiveTargetHz > 0.0 ? 200u : 0u;
    engine::FramePacer framePacer(
        {.targetFrameRateHz = effectiveTargetHz, .spinWindowMicroseconds = pacerSpinWindowMicroseconds});
    framePacer.reset();
    if (pacerSuppressedByVsync)
    {
        std::cout << "demo_planet: max_fps " << maxFps
                  << " left to vsync - presentation is already pacing the frame loop\n";
    }

    // Warmup frames precede the rendered run; both are bounded by their own
    // counter (warmup ends early on a fully built visible set), so the loop
    // runs until the warmup budget is spent AND every requested frame is
    // rendered - or until the window closes in interactive mode.
    for (std::uint64_t frameIndex = 0; runUntilClosed || warmupRemaining > 0 || settleWindowOpen || framesRendered < totalFrames; ++frameIndex)
    {
        frameTiming.beginFrame();
        windowCapability.pollEvents(windowCapability.self);

        demo::harness::drainWindowEvents(windowCapability, windowHandle, eventBuffer,
            [&](const window::WindowEvent &event)
            {
                if (event.kind == window::WindowEventKind::CloseRequested)
                {
                    windowCapability.requestClose(windowCapability.self, windowHandle);
                }
                else if (event.kind == window::WindowEventKind::FramebufferResized)
                {
                    if (event.framebufferWidth != 0 && event.framebufferHeight != 0)
                    {
                        framebufferWidth  = event.framebufferWidth;
                        framebufferHeight = event.framebufferHeight;
                        renderCapability.setViewport(renderCapability.self, rendererHandle, framebufferWidth, framebufferHeight);
                    }
                }
                else
                {
                    cameraInput.handleWindowEvent(event);
                }
            });

        if (windowCapability.shouldClose(windowCapability.self, windowHandle))
        {
            std::cout << "demo_planet: window close requested\n";
            break;
        }
        if (windowCapability.isKeyDown != nullptr
            && windowCapability.isKeyDown(windowCapability.self, windowHandle, window::Key::Escape))
        {
            std::cout << "demo_planet: escape pressed\n";
            break;
        }

        const auto   now               = std::chrono::steady_clock::now();
        const double wallDeltaSeconds = std::min(0.25, std::chrono::duration<double>(now - lastWallClock).count());
        lastWallClock                 = now;
        const double deltaSeconds     = options.frames != 0 ? frameDelta : wallDeltaSeconds;

        camera::CameraInputState cameraSample{};
        if (walkView)
        {
            // The walk controller consumes the input sample and hands the
            // camera its pose; the camera itself runs in Fixed mode and
            // never integrates movement on its own.
            const bool interactive = !options.orbit;
            cameraSample = interactive ? cameraInput.sample() : camera::CameraInputState{};
            walkController->update(cameraSample, interactive, deltaSeconds);
            eye    = walkController->eye();
            target = walkController->target();
            const glm::dvec3 walkUp = walkController->up();
            // Camera-relative composition (see planet_demo_support.hpp): the
            // pose is shifted by the f64 render origin before the f32 cast,
            // so the camera and every model matrix compose eye-relative.
            const glm::dvec3 poseShift = cameraRelativeShift(eye - planetCenter);
            const float poseEye[3] = {static_cast<float>(eye.x - poseShift.x), static_cast<float>(eye.y - poseShift.y),
                static_cast<float>(eye.z - poseShift.z)};
            const float poseTarget[3] = {static_cast<float>(target.x - poseShift.x),
                static_cast<float>(target.y - poseShift.y), static_cast<float>(target.z - poseShift.z)};
            const float poseUp[3] = {
                static_cast<float>(walkUp.x), static_cast<float>(walkUp.y), static_cast<float>(walkUp.z)};
            cameraCapability.setLookAt(cameraCapability.self, cameraHandle, poseEye, poseTarget, poseUp);
        }
        else if (options.orbit)
        {
            // Deterministic automatic orbit mode remains available for captures
            // and headless runs; normal windowed runs use live keyboard/mouse input.
            orbitAngle += deltaSeconds * (options.viewName == "globe" ? 0.05 : 0.10);
            if (options.viewName == "globe")
            {
                const glm::dvec3 axis(0.0, 1.0, 0.0);
                const glm::dvec3 offset = eye - planetCenter;
                const double orbitRadius = glm::length(offset);
                const glm::dvec3 rotated = glm::rotate(glm::dmat4(1.0), orbitAngle, axis) * glm::dvec4(offset, 1.0);
                eye = planetCenter + glm::normalize(rotated) * orbitRadius;
                target = planetCenter;
            }
            else
            {
                const glm::dvec3 localUp = glm::normalize(eye - planetCenter);
                const glm::dvec3 offset = eye - target;
                const glm::dvec3 rotated = glm::rotate(glm::dmat4(1.0), orbitAngle, localUp) * glm::dvec4(offset, 1.0);
                eye = target + rotated;
            }

            const glm::dvec3 lookUp = macroView ? glm::dvec3(0.0, 1.0, 0.0)
                                                : glm::normalize(eye - planetCenter);
            // Camera-relative composition (see planet_demo_support.hpp): the
            // pose is shifted by the f64 render origin before the f32 cast,
            // so the camera and every model matrix compose eye-relative.
            const glm::dvec3 poseShift = cameraRelativeShift(eye - planetCenter);
            const float poseEye[3] = {static_cast<float>(eye.x - poseShift.x), static_cast<float>(eye.y - poseShift.y),
                static_cast<float>(eye.z - poseShift.z)};
            const float poseTarget[3] = {static_cast<float>(target.x - poseShift.x),
                static_cast<float>(target.y - poseShift.y), static_cast<float>(target.z - poseShift.z)};
            float poseUp[3] = {static_cast<float>(lookUp.x), static_cast<float>(lookUp.y), static_cast<float>(lookUp.z)};
            if (macroView
                && glm::abs(glm::dot(glm::normalize(eye - planetCenter), glm::dvec3(0.0, 1.0, 0.0))) > 0.99)
            {
                poseUp[0] = 0.05f;
                poseUp[1] = 0.9987f;
                poseUp[2] = 0.0f;
            }
            cameraCapability.setLookAt(cameraCapability.self, cameraHandle, poseEye, poseTarget, poseUp);
        }
        else
        {
            cameraSample = cameraInput.sample();
        }
        cameraCapability.update(cameraCapability.self, cameraHandle, &cameraSample, deltaSeconds);

        camera::ViewState viewState{};
        cameraCapability.viewState(cameraCapability.self, cameraHandle, framebufferWidth, framebufferHeight, &viewState);
        renderer::CameraMatrices cameraMatrices{};
        std::memcpy(cameraMatrices.view, viewState.view, sizeof(cameraMatrices.view));
        std::memcpy(cameraMatrices.projection, viewState.projection, sizeof(cameraMatrices.projection));
        std::memcpy(cameraMatrices.position, viewState.position, sizeof(cameraMatrices.position));
        renderCapability.setCamera(renderCapability.self, rendererHandle, &cameraMatrices);

        // Advance the planet LOD stream for this eye and refresh GPU meshes.
        // During warmup frames the renderer stays idle: every worker builds
        // chunks at full speed instead of competing with rasterizer tile
        // tasks, so deep LOD views start their first RENDERED frame with the
        // visible set already complete (no black-hole screenshots).
        // The streamer follows the REAL eye (f64), not the camera-relative
        // view state: the view state carries the shifted pose whenever
        // camera-relative composition is active.
        const double eyeWorld[3] = {eye.x, eye.y, eye.z};
        planetCapability->updatePlanet(planetCapability->self, planetHandle, eyeWorld);
        if (warmupRemaining > 0 || settleWindowOpen)
        {
            // Warmup frames exist to give the build pool wall-clock time.
            // With pacing disabled (max_fps 0, the headless default) they
            // would otherwise spin through the budget in microseconds - a
            // static view's update is a handful of fast-path checks - and
            // the pool would never see the machine. Warmup applies its own
            // floor interval in that case, and ends early once the visible
            // set is fully built (the streaming pool is idle), so a large
            // warmup budget costs nothing on an already-complete view. The
            // --settle-seconds window suspends the budget's consumption for
            // its duration: warmup then runs until the visible set is
            // complete or the window closes, whichever comes first, and the
            // frame budget (with its exhaustion warning) resumes after it.
            // Finished meshes also upload to the renderer during warmup:
            // the first RENDERED frame then draws with the whole visible
            // set already resident instead of spending its own frame budget
            // on the initial upload wave. The scatter system joins the same
            // pre-upload: its first rebuild carries the vegetation and
            // sea-state meshes (the next-largest first-frame upload after
            // the chunk wave). The delta stays 0 so every time-driven
            // element - bird flight, foam churn, mist re-roll - holds the
            // exact phase the first rendered frame expects, and the settle
            // iterations only refresh vertices the rendered frames would
            // have refreshed anyway.
            chunkRenderer.updateVisibleChunks();
            scatterSystem.update(eye - planetCenter,
                environment.sun.enabled
                    ? glm::dvec3(sunDirection.x, sunDirection.y, sunDirection.z)
                    : glm::dvec3(0.0),
                environment.sun.enabled, 0.0);
            planet::PlanetStatistics warmStats{};
            // The completion hold (see settleHoldIterations): fully-built
            // snapshots must repeat before warmup trusts them, and the held
            // iterations still run the pre-upload path above, so the visible
            // set stays resident while the wave trough passes.
            bool warmupSettled = false;
            if (planetCapability->copyStatistics(planetCapability->self, planetHandle, &warmStats)
                && warmStats.chunksVisible > 0 && warmStats.chunksBuilding == 0
                && warmStats.chunksReady >= warmStats.chunksVisible)
            {
                if (++settleHoldProgress >= settleHoldIterations)
                {
                    warmupSettled = true;
                }
            }
            else
            {
                settleHoldProgress = 0;
            }
            if (warmupSettled)
            {
                warmupRemaining   = 0;
                settleWindowOpen = false;
            }
            else if (settleWindowOpen)
            {
                if (std::chrono::duration<double>(std::chrono::steady_clock::now() - settleStart).count()
                    >= options.settleSeconds)
                {
                    settleWindowOpen = false;
                    warmupRemaining = 0;
                    warmupExhausted = true;
                    (void)planetCapability->copyStatistics(
                        planetCapability->self, planetHandle, &warmupExhaustionStats);
                }
            }
            else
            {
                --warmupRemaining;
                if (warmupRemaining == 0)
                {
                    warmupExhausted = true;
                    (void)planetCapability->copyStatistics(
                        planetCapability->self, planetHandle, &warmupExhaustionStats);
                }
            }
            (void)framePacer.pace(&host.kernel().profiler());
            if (effectiveTargetHz <= 0.0)
            {
                // One millisecond keeps the loop yield-bound instead of
                // sleep-bound: the build pool converges on worker throughput,
                // not on the warmup cadence (4 ms capped a 2-core machine at
                // ~200 chunk deliveries per second while the pool builds at
                // its own pace beneath that).
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            continue;
        }
        frameTiming.markUpdateDone();
        // The orbital clock (moon_orbital_period_seconds) advances only on
        // RENDERED frames: the warmup and settle branches below spin through
        // tens of thousands of build-only iterations while streaming lands,
        // and letting those advance the companion would fling it through
        // whole orbits between capture frames - its position must be a pure
        // function of the rendered frame count (and stay anchored near the
        // phase-0 sun-relative placement the module's model intends).
        simSeconds += deltaSeconds;
        chunkRenderer.updateVisibleChunks();

        // Sky backdrop: the dome follows the eye (placed, oriented and
        // scaled for this frame's pose) and reports the horizon-haze clear
        // color; space views keep a black clear color, no dome, and the
        // atmosphere limb shell instead.
        const glm::dvec3 localUp = walkView && walkController ? walkController->up()
            : glm::normalize(eye - planetCenter);
        const glm::dvec3 eyePlanetLocal = eye - planetCenter;
        float skyClearColor[3] = {0.0f, 0.0f, 0.0f};
        chunkRenderer.updateSkyDome(eyePlanetLocal, localUp,
            glm::dvec3(sunDirection.x, sunDirection.y, sunDirection.z), environment.sun.enabled, nearZ, farZ,
            skyClearColor);
        chunkRenderer.updateAtmosphereShell(eyePlanetLocal,
            glm::dvec3(sunDirection.x, sunDirection.y, sunDirection.z), environment.sun.enabled);
        chunkRenderer.updateAurora(eyePlanetLocal);
        chunkRenderer.updateStarDome(eyePlanetLocal, farZ);
        chunkRenderer.updateSunDisc(eyePlanetLocal, localUp,
            glm::dvec3(sunDirection.x, sunDirection.y, sunDirection.z), environment.sun.color,
            environment.sun.enabled, farZ);
        const planet::Vec3 moonDirectionThisFrame = planet::moonDirection(params, simSeconds);
        chunkRenderer.updateMoon(eyePlanetLocal, localUp,
            glm::dvec3(moonDirectionThisFrame.x, moonDirectionThisFrame.y, moonDirectionThisFrame.z),
            environment.sun.enabled, farZ);
        scatterSystem.update(eyePlanetLocal,
            environment.sun.enabled ? glm::dvec3(sunDirection.x, sunDirection.y, sunDirection.z) : glm::dvec3(0.0),
            environment.sun.enabled, deltaSeconds);
        renderer::FrameParameters frameParameters{};
        frameParameters.clearColor[0] = skyClearColor[0];
        frameParameters.clearColor[1] = skyClearColor[1];
        frameParameters.clearColor[2] = skyClearColor[2];
        frameParameters.clearColor[3] = 1.0f;

        // The sun tint the sky dome carries this frame (same formula the
        // dome update uses) feeds the per-chunk aerial perspective.
        const double sunUpForSky = environment.sun.enabled
                && planet::length(sunDirection) > 1.0e-9
            ? planet::dot(sunDirection * (1.0 / planet::length(sunDirection)),
                planet::Vec3{localUp.x, localUp.y, localUp.z})
            : 1.0;
        const planet::Vec3 sunTint = planet::sunSkyTint(params, sunUpForSky, environment.sun.enabled);

        // Direct sun: the scene's [pbr] sun carries the atmosphere's
        // transmittance tint this frame (see planet::sunDirectTint), so
        // terrain, water and specular glints warm and redden toward the
        // horizon the way the sun's own light does, staying in one spectral
        // story with the disc and the sky. The eye's atmosphere weight fades
        // the curve back to white for space views, where no air stands
        // between the sun and the camera. The pristine environment copy
        // stays untouched, so every frame blends from the configured color
        // (a static camera pushes one identical environment and the pbr
        // module's version gate stops the churn there).
        if (pbrCapability != nullptr)
        {
            const double eyeAltitude
                = planet::length(planet::Vec3{eyePlanetLocal.x, eyePlanetLocal.y, eyePlanetLocal.z})
                    - params.radiusMeters;
            const double airWeight = planet::atmosphereWeight(params, eyeAltitude);
            const planet::Vec3 directTint = planet::sunDirectTint(params, sunUpForSky, environment.sun.enabled);
            pbr::PbrEnvironment frameEnvironment = environment;
            frameEnvironment.sun.color[0] = static_cast<float>(environment.sun.color[0]
                * (1.0 + (directTint.x - 1.0) * airWeight));
            frameEnvironment.sun.color[1] = static_cast<float>(environment.sun.color[1]
                * (1.0 + (directTint.y - 1.0) * airWeight));
            frameEnvironment.sun.color[2] = static_cast<float>(environment.sun.color[2]
                * (1.0 + (directTint.z - 1.0) * airWeight));
            (void)pbrCapability->setEnvironment(pbrCapability->self, &frameEnvironment);
        }

        const glm::mat4 viewProjection = glm::make_mat4(viewState.projection) * glm::make_mat4(viewState.view);
        chunkRenderer.rebuildDrawCommands(viewProjection, eyePlanetLocal, sunTint, cameraRelativeShift(eyePlanetLocal));
        const auto &drawCommands = chunkRenderer.drawCommands();
        frameTiming.markUploadDone();

        renderCapability.beginFrame(renderCapability.self, rendererHandle, &frameParameters);
        renderCapability.drawMeshes(renderCapability.self, rendererHandle, drawCommands.data(),
            static_cast<std::uint32_t>(drawCommands.size()));
        renderCapability.endFrame(renderCapability.self, rendererHandle);
        frameTiming.markDrawDone();
        ++framesRendered;
        if (framesRendered % 100 == 0)
        {
            renderer::FrameStatistics frameStats{};
            renderCapability.frameStatistics(renderCapability.self, rendererHandle, &frameStats);
            std::cout << "demo_planet: raster stats - submitted " << frameStats.trianglesSubmitted
                      << ", rasterized " << frameStats.trianglesRasterized << "\n";
        }

        // Both stats reports cadence on framesRendered (warmup excluded):
        // the raster stats above use the same counter, so the frame numbers
        // in the log agree across reports.
        if (options.statsEveryFrames != 0 && framesRendered % options.statsEveryFrames == 0)
        {
            planet::PlanetStatistics statistics{};
            if (planetCapability->copyStatistics(planetCapability->self, planetHandle, &statistics))
            {
                std::cout << "demo_planet: frame " << frameIndex << " | chunks visible " << statistics.chunksVisible
                          << ", ready " << statistics.chunksReady << ", building " << statistics.chunksBuilding
                          << ", fallback " << statistics.chunksFallback
                          << ", cached " << statistics.chunksReadyTotal << ", triangles " << statistics.triangleCount
                          << ", depth " << statistics.octreeDepthUsed
                          << ", update " << statistics.lastUpdateMilliseconds << " ms\n";
            }
        }

        // Screenshot frames count RENDERED frames (warmup excluded, 0-based),
        // so the default last-frame index lands on the last rendered frame at
        // any warmup depth. framesRendered is already the COUNT of rendered
        // frames here; rendered frame i has completed when it equals i + 1.
        if (!screenshotTaken && !options.screenshotPath.empty()
            && framesRendered == options.screenshotFrame + 1)
        {
            std::string screenshotWrittenPath;
            screenshotTaken = demo::harness::exportScreenshot(context,
                windowCapability,
                windowHandle,
                renderCapability,
                rendererHandle,
                options.screenshotPath,
                screenshotWrittenPath);
            if (screenshotTaken)
            {
                std::cout << "demo_planet: screenshot written to " << screenshotWrittenPath << "\n";
            }
            else
            {
                std::cerr << "demo_planet: screenshot export failed for backend '" << windowBackendName << "'\n";
            }
        }
        (void)framePacer.pace(&host.kernel().profiler());
    }

    // ---- teardown -----------------------------------------------------------------------
    std::cout << frameTiming.summary();
    if (!options.frameTimesPath.empty())
    {
        if (frameTiming.writeCsv(options.frameTimesPath))
        {
            std::cout << "demo_planet: frame timings written to " << options.frameTimesPath << "\n";
        }
        else
        {
            std::cerr << "demo_planet: could not write frame timings to " << options.frameTimesPath << "\n";
        }
    }
    scatterSystem.shutdown();
    chunkRenderer.destroy();
    // Final statistics must be read while the planet (and its module) still
    // exist: host.stop() below destroys the module, and any capability call
    // after that would dereference freed memory.
    planet::PlanetStatistics finalStatistics{};
    if (planetCapability->copyStatistics(planetCapability->self, planetHandle, &finalStatistics))
    {
        std::cout << "demo_planet: totals - chunks cached " << finalStatistics.chunksReadyTotal << ", triangles "
                  << finalStatistics.triangleCount << ", vertices " << finalStatistics.verticesTotal << "\n";
        if (warmupExhausted)
        {
            const bool completedLater = finalStatistics.chunksVisible > 0
                && finalStatistics.chunksReady >= finalStatistics.chunksVisible;
            std::cout << "demo_planet: warning - the warmup budget ran out with "
                      << warmupExhaustionStats.chunksReady << "/" << warmupExhaustionStats.chunksVisible
                      << " visible chunks ready; the first rendered frames and any early screenshot"
                      << " may show streaming gaps (raise --warmup for a complete capture)";
            if (completedLater)
            {
                std::cout << "; the set finished building during the rendered frames (now "
                          << finalStatistics.chunksReady << "/" << finalStatistics.chunksVisible << " ready)";
            }
            std::cout << "\n";
        }
    }
    planetCapability->destroyPlanet(planetCapability->self, planetHandle);
    cameraCapability.destroyCamera(cameraCapability.self, cameraHandle);
    renderCapability.destroyRenderer(renderCapability.self, rendererHandle);
    windowCapability.destroyWindow(windowCapability.self, windowHandle);
    host.stop();
    std::cout << "demo_planet: stopped cleanly after " << framesRendered << " frames\n";
    return 0;
}
