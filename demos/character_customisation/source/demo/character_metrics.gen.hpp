#ifndef ENGINE_DEMO_CHARACTER_METRICS_GEN_HPP
#define ENGINE_DEMO_CHARACTER_METRICS_GEN_HPP
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
namespace demo
{
namespace metrics
{
using glm::quat;
using glm::vec2;
using glm::vec3;
inline const vec3 kRangerTorsoNeckRing{0.00000f, 0.58895f, 0.01687f};
inline const float kRangerTorsoNeckRadius = 0.05321f;
inline const vec3 kRangerTorsoSocketShoulderLeft{0.19190f, 0.50640f, -0.02240f};
inline const vec3 kRangerTorsoSocketShoulderRight{-0.19190f, 0.50640f, -0.02240f};
inline const vec3 kRangerTorsoSocketHipLeft{0.09060f, 0.02210f, 0.00700f};
inline const vec3 kRangerTorsoSocketHipRight{-0.09060f, 0.02210f, 0.00700f};
inline const vec3 kRangerTorsoSocketTail{0.00000f, 0.06210f, -0.14295f};
inline const vec3 kRangerTorsoSocketWingLeft{-0.02000f, 0.46895f, -0.08359f};
inline const vec3 kRangerTorsoSocketWingRight{0.02000f, 0.46895f, -0.08359f};
inline const vec3 kRangerTorsoSocketBelt{0.00000f, 0.22870f, 0.04690f};
inline const vec3 kRangerTorsoSocketPadLeft{0.19190f, 0.55640f, -0.02240f};
inline const vec3 kRangerTorsoSocketPadRight{-0.19190f, 0.55640f, -0.02240f};
inline const quat kRangerTorsoSocketTailRotation = glm::quat(0.322874f, 0.946442f, 0.000000f, 0.000000f);
inline const vec3 kBareTorsoNeckRing{0.00000f, 0.65083f, 0.02124f};
inline const float kBareTorsoNeckRadius = 0.06243f;
inline const vec3 kBareTorsoSocketShoulderLeft{0.21201f, 0.50640f, -0.02240f};
inline const vec3 kBareTorsoSocketShoulderRight{-0.21201f, 0.50640f, -0.02240f};
inline const vec3 kBareTorsoSocketHipLeft{0.11431f, 0.02210f, 0.00700f};
inline const vec3 kBareTorsoSocketHipRight{-0.11431f, 0.02210f, 0.00700f};
inline const vec3 kBareTorsoSocketTail{0.00000f, 0.06210f, -0.08516f};
inline const vec3 kBareTorsoSocketWingLeft{-0.02000f, 0.53083f, -0.09572f};
inline const vec3 kBareTorsoSocketWingRight{0.02000f, 0.53083f, -0.09572f};
inline const vec3 kBareTorsoSocketBelt{0.00000f, 0.22870f, 0.04690f};
inline const vec3 kBareTorsoSocketPadLeft{0.21201f, 0.55640f, -0.02240f};
inline const vec3 kBareTorsoSocketPadRight{-0.21201f, 0.55640f, -0.02240f};
inline const quat kBareTorsoSocketTailRotation = glm::quat(0.322874f, 0.946442f, 0.000000f, 0.000000f);
inline const float kBareLegFootY = -0.96789f;
inline const float kRangerLegFootY = -0.96939f;
inline const float kBareArmSpan = 0.71659f;
inline const vec3 kBareArmBracerSocket{-0.03540f, -0.29624f, -0.01622f};
inline const vec3 kRangerArmBracerSocket{-0.03554f, -0.30940f, -0.00395f};
inline const vec3 kBeltSocket{-0.00311f, 0.23145f, 0.05061f};
struct SpeciesMetrics
{
    const char *id;
    const char *display;
    vec3 torsoMount;
    quat torsoMountRotation;
    float stumpRadius;
    vec3 tailMount;
    quat tailMountRotation;
    float tailStumpRadius;
    vec3 withers;
    float groundY;
    float lengthZ;
    float taurScale;
    vec3 earSocketLeft;
    quat earRotationLeft;
    vec3 earSocketRight;
    quat earRotationRight;
    bool hasEarSockets;
    float tailRootRadius;
};
inline const SpeciesMetrics kSpecies[11] = {
    {"fox", "Fox", vec3{0.00453f, 0.00542f, 0.48577f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), 0.11009f, vec3{0.00000f, 0.04951f, -0.31151f}, glm::quat(0.962377f, -0.271718f, -0.000000f, 0.000000f), 0.04944f, vec3{0.00000f, 0.16041f, 0.58753f}, -0.58412f, 1.00228f, 0.31899f, vec3{0.03609f, 0.09317f, 0.16129f}, glm::quat(0.844096f, 0.315458f, 0.000000f, -0.433577f), vec3{-0.03609f, 0.09317f, 0.16129f}, glm::quat(0.853657f, 0.299786f, 0.000000f, 0.425910f), true, 0.08547f},
    {"shibainu", "Shiba", vec3{0.00000f, 0.11070f, 0.33358f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), 0.09671f, vec3{-0.09008f, -0.36638f, -0.19804f}, glm::quat(0.768736f, 0.101992f, 0.631381f, 0.000000f), 0.03396f, vec3{0.00000f, 0.22042f, 0.35634f}, -0.40516f, 0.74098f, 0.23294f, vec3{0.03155f, 0.15763f, 0.09864f}, glm::quat(0.758928f, 0.568732f, 0.000000f, -0.317130f), vec3{-0.03155f, 0.15763f, 0.09864f}, glm::quat(0.753617f, 0.581450f, 0.000000f, 0.306556f), true, 0.11109f},
    {"husky", "Husky", vec3{-0.00000f, 0.17846f, 0.44409f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), 0.11519f, vec3{-0.11300f, -0.48734f, -0.26401f}, glm::quat(0.740345f, -0.109395f, 0.663267f, 0.000000f), 0.04691f, vec3{0.00000f, 0.33873f, 0.50121f}, -0.54620f, 0.96514f, 0.31392f, vec3{0.03419f, 0.15225f, 0.11469f}, glm::quat(0.860362f, 0.384093f, 0.000000f, -0.335036f), vec3{-0.03419f, 0.15225f, 0.11469f}, glm::quat(0.826525f, 0.401212f, 0.000000f, 0.394823f), true, 0.11497f},
    {"wolf", "Wolf", vec3{-0.00000f, 0.03911f, 0.55728f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), 0.14500f, vec3{0.00000f, -0.01825f, -0.39109f}, glm::quat(0.986193f, -0.165602f, -0.000000f, 0.000000f), 0.05862f, vec3{0.00000f, 0.19323f, 0.66881f}, -0.73963f, 1.20347f, 0.39152f, vec3{0.04727f, 0.08952f, 0.12365f}, glm::quat(0.884142f, 0.432654f, 0.000000f, -0.176360f), vec3{-0.04722f, 0.08952f, 0.12365f}, glm::quat(0.892744f, 0.407630f, 0.000000f, 0.191954f), true, 0.10275f},
    {"donkey", "Donkey", vec3{0.00000f, 0.12633f, 0.57078f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), 0.10437f, vec3{0.10655f, -0.58947f, -0.28912f}, glm::quat(0.512460f, -0.260147f, 0.818357f, 0.000000f), 0.03245f, vec3{0.00000f, 0.25268f, 0.62353f}, -0.62092f, 1.06473f, 0.30312f, vec3{0.02507f, 0.15713f, 0.14313f}, glm::quat(0.959505f, 0.265047f, 0.000000f, -0.095392f), vec3{-0.02507f, 0.15713f, 0.14313f}, glm::quat(0.958999f, 0.266196f, 0.000000f, 0.097264f), true, 0.17637f},
    {"cow", "Cow", vec3{0.00000f, -0.03972f, 0.92534f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), 0.14500f, vec3{0.13716f, -0.74648f, -0.17637f}, glm::quat(0.619124f, -0.741055f, -0.259851f, 0.000000f), 0.04206f, vec3{0.00000f, 0.18419f, 0.72700f}, -0.78409f, 1.49347f, 0.22497f, vec3{0.00000f, 0.10000f, 0.00000f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), vec3{-0.00000f, 0.10000f, 0.00000f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), false, 0.14523f},
    {"deer", "Deer", vec3{0.00000f, 0.15846f, 0.47976f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), 0.08127f, vec3{0.09265f, -0.52329f, -0.22685f}, glm::quat(0.590491f, -0.748435f, -0.301935f, 0.000000f), 0.02869f, vec3{0.00000f, 0.24975f, 0.52607f}, -0.55339f, 0.92327f, 0.25816f, vec3{0.02316f, 0.20527f, 0.09457f}, glm::quat(0.889344f, 0.181443f, 0.000000f, -0.419696f), vec3{-0.02316f, 0.20527f, 0.09457f}, glm::quat(0.887063f, 0.188202f, 0.000000f, 0.421543f), true, 0.12181f},
    {"bull", "Bull", vec3{0.00000f, -0.04175f, 0.88965f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), 0.14500f, vec3{0.12990f, -0.70696f, -0.16703f}, glm::quat(0.619124f, -0.741055f, -0.259851f, 0.000000f), 0.03983f, vec3{0.00000f, 0.17444f, 0.68851f}, -0.74258f, 1.41440f, 0.21306f, vec3{0.00000f, 0.10000f, 0.00000f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), vec3{-0.00000f, 0.10000f, 0.00000f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), false, 0.14534f},
    {"alpaca", "Alpaca", vec3{-0.00942f, 0.20191f, 0.36356f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), 0.08939f, vec3{0.00431f, 0.03884f, -0.20686f}, glm::quat(0.994001f, 0.108888f, -0.010221f, 0.000000f), 0.03294f, vec3{0.00000f, 0.28710f, 0.38462f}, -0.40196f, 0.75210f, 0.20711f, vec3{0.01001f, 0.33050f, 0.07594f}, glm::quat(0.815199f, 0.523552f, 0.000000f, -0.247676f), vec3{-0.01745f, 0.32982f, 0.07814f}, glm::quat(0.748468f, 0.655989f, 0.000000f, 0.097340f), true, 0.15406f},
    {"horse", "Horse", vec3{-0.00000f, 0.22891f, 0.68683f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), 0.11814f, vec3{-0.00000f, 0.08596f, -0.34116f}, glm::quat(0.991654f, 0.128926f, 0.000000f, 0.000000f), 0.05397f, vec3{0.00000f, 0.37866f, 0.75540f}, -0.76202f, 1.23699f, 0.30128f, vec3{0.02842f, 0.19948f, 0.16792f}, glm::quat(0.828471f, 0.544618f, 0.000000f, -0.130485f), vec3{-0.02655f, 0.19948f, 0.16792f}, glm::quat(0.834478f, 0.532829f, 0.000000f, 0.140497f), true, 0.10524f},
    {"stag", "Stag", vec3{0.00000f, 0.17293f, 0.56463f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), 0.12947f, vec3{-0.11135f, -0.59072f, -0.27498f}, glm::quat(0.711052f, -0.700369f, 0.062358f, 0.000000f), 0.03281f, vec3{0.00000f, 0.30740f, 0.61300f}, -0.61234f, 1.08354f, 0.27432f, vec3{0.00000f, 0.10000f, 0.00000f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), vec3{-0.00000f, 0.10000f, 0.00000f}, glm::quat(1.000000f, 0.000000f, 0.000000f, 0.000000f), false, 0.12290f},
};
inline constexpr int kSpeciesCount = 11;
} // namespace metrics
} // namespace demo
#endif // ENGINE_DEMO_CHARACTER_METRICS_GEN_HPP
