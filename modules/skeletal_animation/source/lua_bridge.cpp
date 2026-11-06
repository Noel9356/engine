#include "lua_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>
#include <utility>

namespace engine::skeletal_animation
{
namespace
{
using script_runtime::ScriptNativeFunction;
using script_runtime::ScriptRuntimeHandle;
using script_runtime::ScriptValue;
using script_runtime::ScriptValueType;
using Bridge = SkeletalAnimationLuaBridgeContext;

bool number(const ScriptValue *args, std::uint32_t count, std::uint32_t index, double &out)
{
    if (index >= count || args[index].type != ScriptValueType::Number || !std::isfinite(args[index].numberValue))
        return false;
    out = args[index].numberValue;
    return true;
}

bool uint32Value(const ScriptValue *args, std::uint32_t count, std::uint32_t index, std::uint32_t &out)
{
    double value = 0.0;
    if (!number(args, count, index, value) || value < 0.0
        || value > static_cast<double>(std::numeric_limits<std::uint32_t>::max()) || std::floor(value) != value)
        return false;
    out = static_cast<std::uint32_t>(value);
    return true;
}

bool int32Value(const ScriptValue *args, std::uint32_t count, std::uint32_t index, std::int32_t &out)
{
    double value = 0.0;
    if (!number(args, count, index, value) || value < static_cast<double>(std::numeric_limits<std::int32_t>::min())
        || value > static_cast<double>(std::numeric_limits<std::int32_t>::max()) || std::floor(value) != value)
        return false;
    out = static_cast<std::int32_t>(value);
    return true;
}

bool floatValue(const ScriptValue *args, std::uint32_t count, std::uint32_t index, float &out)
{
    double value = 0.0;
    if (!number(args, count, index, value) || value < -std::numeric_limits<float>::max()
        || value > std::numeric_limits<float>::max())
        return false;
    out = static_cast<float>(value);
    return true;
}

bool boolean(const ScriptValue *args, std::uint32_t count, std::uint32_t index, bool &out)
{
    if (index >= count || args[index].type != ScriptValueType::Boolean)
        return false;
    out = args[index].booleanValue;
    return true;
}

bool stringValue(const ScriptValue *args, std::uint32_t count, std::uint32_t index, const char *&out)
{
    if (index >= count || args[index].type != ScriptValueType::String || args[index].stringData == nullptr)
        return false;
    out = args[index].stringData;
    return true;
}

void resultNumber(ScriptValue *out, std::uint32_t index, double value)
{
    out[index].type = ScriptValueType::Number;
    out[index].numberValue = value;
}
void resultBoolean(ScriptValue *out, std::uint32_t index, bool value)
{
    out[index].type = ScriptValueType::Boolean;
    out[index].booleanValue = value;
}
void resultNil(ScriptValue *out, std::uint32_t index)
{
    out[index] = {};
    out[index].type = ScriptValueType::Nil;
}
void resultError(ScriptValue *out, std::uint32_t index, AnimationError error)
{
    resultNumber(out, index, static_cast<std::uint32_t>(error));
}
void resultHandle(ScriptValue *out, std::uint32_t index, std::uint32_t handle)
{
    if (handle == 0) resultNil(out, index); else resultNumber(out, index, handle);
}

bool allocateBuilderHandle(Bridge &bridge, std::uint32_t &out)
{
    if (bridge.nextBuilder == 0 || bridge.nextBuilder > std::numeric_limits<std::uint32_t>::max())
        return false;
    out = static_cast<std::uint32_t>(bridge.nextBuilder++);
    return true;
}

AnimationInterpolation parseInterpolation(const char *value, bool &valid)
{
    valid = true;
    if (value == nullptr || value[0] == '\0' || std::string_view(value) == "default")
        return AnimationInterpolation::Default;
    if (std::string_view(value) == "slerp") return AnimationInterpolation::Slerp;
    if (std::string_view(value) == "nlerp") return AnimationInterpolation::Nlerp;
    if (std::string_view(value) == "step") return AnimationInterpolation::Step;
    valid = false;
    return AnimationInterpolation::Default;
}

bool beginSkeleton(void *userData,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*outCount)
{
    auto &b=*static_cast<Bridge*>(userData);std::uint32_t count=0;if(n!=1||cap<1||!uint32Value(a,n,0,count))return false;
    std::lock_guard lock(b.mutex);
    if(count==0||count>b.maximumJoints||b.skeletonBuilders.size()>=b.maximumSkeletonBuilders
        ||count>b.maximumPendingJoints-std::min(b.pendingJoints,b.maximumPendingJoints)){resultNil(r,0);*outCount=1;return true;}
    std::uint32_t handle=0;if(!allocateBuilderHandle(b,handle)){resultNil(r,0);*outCount=1;return true;}
    Bridge::SkeletonBuilder builder;builder.joints.resize(count);builder.assigned.resize(count,0);b.skeletonBuilders.emplace(handle,std::move(builder));b.pendingJoints+=count;resultNumber(r,0,handle);*outCount=1;return true;
}

bool setJoint(void *userData,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*outCount)
{
    auto &b=*static_cast<Bridge*>(userData);std::uint32_t handle=0,index=0;std::int32_t parent=-1;JointDescription joint{};
    if(n!=13||cap<1||!uint32Value(a,n,0,handle)||!uint32Value(a,n,1,index)||!int32Value(a,n,2,parent))return false;
    std::uint32_t k=3;for(float &v:joint.translation)if(!floatValue(a,n,k++,v))return false;for(float &v:joint.rotation)if(!floatValue(a,n,k++,v))return false;for(float &v:joint.scale)if(!floatValue(a,n,k++,v))return false;
    bool ok=false;{
        std::lock_guard lock(b.mutex);auto found=b.skeletonBuilders.find(handle);if(found!=b.skeletonBuilders.end()&&index<found->second.joints.size()&&parent>=-1&&parent<static_cast<std::int32_t>(found->second.joints.size())&&parent!=static_cast<std::int32_t>(index)){
            joint.parentIndex=parent;found->second.joints[index]=joint;found->second.assigned[index]=1;ok=true;
        }
    }resultBoolean(r,0,ok);*outCount=1;return true;
}

bool finishSkeleton(void *userData,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*outCount)
{
    auto &b=*static_cast<Bridge*>(userData);std::uint32_t handle=0;if(n!=1||cap<2||!uint32Value(a,n,0,handle))return false;
    Bridge::SkeletonBuilder builder;{
        std::lock_guard lock(b.mutex);auto found=b.skeletonBuilders.find(handle);if(found==b.skeletonBuilders.end()){resultNil(r,0);resultError(r,1,AnimationError::InvalidHandle);*outCount=2;return true;}
        if(std::find(found->second.assigned.begin(),found->second.assigned.end(),0)!=found->second.assigned.end()){resultNil(r,0);resultError(r,1,AnimationError::InvalidDescription);*outCount=2;return true;}
        b.pendingJoints-=std::min<std::uint64_t>(b.pendingJoints,found->second.joints.size());
        builder=std::move(found->second);b.skeletonBuilders.erase(found);
    }
    SkeletonDescription d{builder.joints.data(),static_cast<std::uint32_t>(builder.joints.size())};AnimationError error=AnimationError::None;
    std::lock_guard apiLock(b.apiMutex);auto created=b.animation->createSkeleton(b.animation->self,&d,&error);
    resultHandle(r,0,created);resultError(r,1,error);*outCount=2;return true;
}

bool beginClip(void *userData,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*outCount)
{
    auto &b=*static_cast<Bridge*>(userData);const char*name=nullptr;float duration=0;if(n!=2||cap<1||!stringValue(a,n,0,name)||!floatValue(a,n,1,duration))return false;
    std::lock_guard lock(b.mutex);if(name[0]=='\0'||duration<0.0f||b.clipBuilders.size()>=b.maximumClipBuilders){resultNil(r,0);*outCount=1;return true;}
    std::uint32_t handle=0;if(!allocateBuilderHandle(b,handle)){resultNil(r,0);*outCount=1;return true;}
    Bridge::ClipBuilder builder;builder.name=name;builder.durationSeconds=duration;b.clipBuilders.emplace(handle,std::move(builder));resultNumber(r,0,handle);*outCount=1;return true;
}

bool beginTrack(void *userData,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*outCount)
{
    auto &b=*static_cast<Bridge*>(userData);std::uint32_t handle=0,joint=0;const char*mode=nullptr;bool valid=false;
    if(n!=3||cap<1||!uint32Value(a,n,0,handle)||!uint32Value(a,n,1,joint)||!stringValue(a,n,2,mode))return false;
    auto interpolation=parseInterpolation(mode,valid);if(!valid)return false;std::lock_guard lock(b.mutex);auto found=b.clipBuilders.find(handle);
    if(found==b.clipBuilders.end()||found->second.tracks.size()>=b.maximumTracksPerClip||joint>=b.maximumJoints){resultNil(r,0);*outCount=1;return true;}
    const auto index=static_cast<std::uint32_t>(found->second.tracks.size());found->second.tracks.push_back(Bridge::TrackBuilder{.jointIndex=joint,.interpolation=interpolation,.keyframes={}});resultNumber(r,0,index);*outCount=1;return true;
}

bool addKeyframe(void *userData,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*outCount)
{
    auto &b=*static_cast<Bridge*>(userData);std::uint32_t handle=0,track=0;AnimationKeyframe key{};
    if(n!=13||cap<1||!uint32Value(a,n,0,handle)||!uint32Value(a,n,1,track)||!floatValue(a,n,2,key.timeSeconds))return false;
    std::uint32_t k=3;for(float &v:key.translation)if(!floatValue(a,n,k++,v))return false;for(float &v:key.rotation)if(!floatValue(a,n,k++,v))return false;for(float &v:key.scale)if(!floatValue(a,n,k++,v))return false;
    bool ok=false;{std::lock_guard lock(b.mutex);auto found=b.clipBuilders.find(handle);if(found!=b.clipBuilders.end()&&track<found->second.tracks.size()){
        auto&keys=found->second.tracks[track].keyframes;if(keys.size()<b.maximumKeyframesPerTrack
            &&b.pendingKeyframes<b.maximumPendingKeyframes){keys.push_back(key);++b.pendingKeyframes;ok=true;}
    }}resultBoolean(r,0,ok);*outCount=1;return true;
}

bool finishClip(void *userData,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*outCount)
{
    auto &b=*static_cast<Bridge*>(userData);std::uint32_t handle=0;if(n!=1||cap<2||!uint32Value(a,n,0,handle))return false;Bridge::ClipBuilder builder;
    {std::lock_guard lock(b.mutex);auto found=b.clipBuilders.find(handle);if(found==b.clipBuilders.end()){resultNil(r,0);resultError(r,1,AnimationError::InvalidHandle);*outCount=2;return true;}
     if(found->second.tracks.empty()||std::any_of(found->second.tracks.begin(),found->second.tracks.end(),[](const auto&t){return t.keyframes.empty();})){resultNil(r,0);resultError(r,1,AnimationError::InvalidDescription);*outCount=2;return true;}
     std::uint64_t keyCount=0;for(const auto&track:found->second.tracks)keyCount+=track.keyframes.size();
     b.pendingKeyframes-=std::min(b.pendingKeyframes,keyCount);
     builder=std::move(found->second);b.clipBuilders.erase(found);}
    std::vector<AnimationTrackDescription> tracks;tracks.reserve(builder.tracks.size());for(const auto&track:builder.tracks){tracks.push_back(AnimationTrackDescription{.jointIndex=track.jointIndex,.keyframes=track.keyframes.data(),.keyframeCount=static_cast<std::uint32_t>(track.keyframes.size()),.interpolation=track.interpolation});}
    AnimationClipDescription d{builder.name.c_str(),builder.durationSeconds,tracks.data(),static_cast<std::uint32_t>(tracks.size())};AnimationError error=AnimationError::None;
    std::lock_guard apiLock(b.apiMutex);auto created=b.animation->createClip(b.animation->self,&d,&error);resultHandle(r,0,created);resultError(r,1,error);*outCount=2;return true;
}

bool cancelBuilder(void *userData,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*outCount)
{
    auto&b=*static_cast<Bridge*>(userData);std::uint32_t handle=0;if(n!=1||cap<1||!uint32Value(a,n,0,handle))return false;std::lock_guard lock(b.mutex);
    bool ok=false;auto skeleton=b.skeletonBuilders.find(handle);if(skeleton!=b.skeletonBuilders.end()){
        b.pendingJoints-=std::min<std::uint64_t>(b.pendingJoints,skeleton->second.joints.size());b.skeletonBuilders.erase(skeleton);ok=true;
    }else{auto clip=b.clipBuilders.find(handle);if(clip!=b.clipBuilders.end()){
        std::uint64_t count=0;for(const auto&track:clip->second.tracks)count+=track.keyframes.size();b.pendingKeyframes-=std::min(b.pendingKeyframes,count);b.clipBuilders.erase(clip);ok=true;
    }}resultBoolean(r,0,ok);*outCount=1;return true;
}

bool destroySkeleton(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*,std::uint32_t,std::uint32_t*o){std::uint32_t h=0;if(n!=1||!uint32Value(a,n,0,h))return false;auto&b=*static_cast<Bridge*>(u);std::lock_guard lock(b.apiMutex);auto&api=*b.animation;api.destroySkeleton(api.self,h);*o=0;return true;}
bool destroyClip(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*,std::uint32_t,std::uint32_t*o){std::uint32_t h=0;if(n!=1||!uint32Value(a,n,0,h))return false;auto&b=*static_cast<Bridge*>(u);std::lock_guard lock(b.apiMutex);auto&api=*b.animation;api.destroyClip(api.self,h);*o=0;return true;}
bool destroyPlayer(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*,std::uint32_t,std::uint32_t*o){std::uint32_t h=0;if(n!=1||!uint32Value(a,n,0,h))return false;auto&b=*static_cast<Bridge*>(u);std::lock_guard lock(b.apiMutex);auto&api=*b.animation;api.destroyPlayer(api.self,h);*o=0;return true;}

bool skeletonJointCount(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*o)
{std::uint32_t h=0,count=0;if(n!=1||cap<1||!uint32Value(a,n,0,h))return false;auto&b=*static_cast<Bridge*>(u);std::lock_guard lock(b.apiMutex);auto&api=*b.animation;if(api.skeletonInfo(api.self,h,&count))resultNumber(r,0,count);else resultNil(r,0);*o=1;return true;}
bool clipDuration(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*o)
{std::uint32_t h=0;float duration=0;if(n!=1||cap<1||!uint32Value(a,n,0,h))return false;auto&b=*static_cast<Bridge*>(u);std::lock_guard lock(b.apiMutex);auto&api=*b.animation;if(api.clipDuration(api.self,h,&duration))resultNumber(r,0,duration);else resultNil(r,0);*o=1;return true;}

bool createPlayer(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*o)
{
    std::uint32_t clip=0;AnimationPlaybackDescription d{};if(n>5||cap<2||!uint32Value(a,n,0,clip))return false;d.clip=clip;
    if (n > 1 && !floatValue(a, n, 1, d.speed)) return false;
    if (n > 2 && !floatValue(a, n, 2, d.initialTimeSeconds)) return false;
    if (n > 3 && !boolean(a, n, 3, d.looping)) return false;
    if (n > 4 && !boolean(a, n, 4, d.playing)) return false;
    AnimationError error=AnimationError::None;auto&b=*static_cast<Bridge*>(u);std::lock_guard lock(b.apiMutex);auto&api=*b.animation;auto h=api.createPlayer(api.self,&d,&error);resultHandle(r,0,h);resultError(r,1,error);*o=2;return true;
}
bool setPlayerClip(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{std::uint32_t p=0,clip=0;bool restart=false;if(n!=3||c<1||!uint32Value(a,n,0,p)||!uint32Value(a,n,1,clip)||!boolean(a,n,2,restart))return false;auto&b=*static_cast<Bridge*>(u);std::lock_guard lock(b.apiMutex);auto&api=*b.animation;resultBoolean(r,0,api.setPlayerClip(api.self,p,clip,restart));*o=1;return true;}
bool setPlayerSpeed(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{std::uint32_t p=0;float v=0;if(n!=2||c<1||!uint32Value(a,n,0,p)||!floatValue(a,n,1,v))return false;auto&b=*static_cast<Bridge*>(u);std::lock_guard lock(b.apiMutex);auto&api=*b.animation;resultBoolean(r,0,api.setPlayerSpeed(api.self,p,v));*o=1;return true;}
bool setPlayerPlaying(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{std::uint32_t p=0;bool v=false;if(n!=2||c<1||!uint32Value(a,n,0,p)||!boolean(a,n,1,v))return false;auto&b=*static_cast<Bridge*>(u);std::lock_guard lock(b.apiMutex);auto&api=*b.animation;resultBoolean(r,0,api.setPlayerPlaying(api.self,p,v));*o=1;return true;}
bool advancePlayer(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{std::uint32_t p=0;float delta=0;if(n!=2||c<1||!uint32Value(a,n,0,p)||!floatValue(a,n,1,delta))return false;auto&b=*static_cast<Bridge*>(u);std::lock_guard lock(b.apiMutex);auto&api=*b.animation;resultBoolean(r,0,api.advancePlayer(api.self,p,delta));*o=1;return true;}
bool playerState(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{std::uint32_t p=0;if(n!=1||c<4||!uint32Value(a,n,0,p))return false;AnimationPlayerState s{};auto&b=*static_cast<Bridge*>(u);std::lock_guard lock(b.apiMutex);auto&api=*b.animation;if(!api.playerState(api.self,p,&s)){resultNil(r,0);*o=1;return true;}resultNumber(r,0,s.timeSeconds);resultNumber(r,1,s.normalizedTime);resultBoolean(r,2,s.playing);resultBoolean(r,3,s.finished);*o=4;return true;}

bool outputPoseJoint(Bridge&b,AnimationPlayerHandle player,SkeletonHandle skeleton,std::uint32_t joint,bool evaluate,ScriptValue*r,std::uint32_t cap,std::uint32_t*outCount)
{
    std::lock_guard apiLock(b.apiMutex);
    std::uint32_t jointCount=0;if(!b.animation->skeletonInfo(b.animation->self,skeleton,&jointCount)||joint>=jointCount){resultNil(r,0);*outCount=1;return true;}
    thread_local std::vector<JointDescription> pose;pose.resize(jointCount);AnimationError error=AnimationError::None;
    const bool ok=evaluate?b.animation->evaluatePlayer(b.animation->self,player,skeleton,pose.data(),jointCount,&error)
        :false;
    if(!evaluate||!ok){resultNil(r,0);*outCount=1;return true;}
    const JointDescription&v=pose[joint];std::uint32_t k=0;for(float x:v.translation)resultNumber(r,k++,x);for(float x:v.rotation)resultNumber(r,k++,x);for(float x:v.scale)resultNumber(r,k++,x);
    *outCount=k;return cap>=k;
}

bool evaluateJoint(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{std::uint32_t p=0,s=0,j=0;if(n!=3||c<10||!uint32Value(a,n,0,p)||!uint32Value(a,n,1,s)||!uint32Value(a,n,2,j))return false;return outputPoseJoint(*static_cast<Bridge*>(u),p,s,j,true,r,c,o);}

bool sampleJoint(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    std::uint32_t clip=0,skeleton=0,joint=0;float time=0;bool looping=true;if(n!=5||c<10||!uint32Value(a,n,0,clip)||!uint32Value(a,n,1,skeleton)||!floatValue(a,n,2,time)||!boolean(a,n,3,looping)||!uint32Value(a,n,4,joint))return false;
    auto&b=*static_cast<Bridge*>(u);std::lock_guard apiLock(b.apiMutex);std::uint32_t jointCount=0;if(!b.animation->skeletonInfo(b.animation->self,skeleton,&jointCount)||joint>=jointCount){resultNil(r,0);*o=1;return true;}
    thread_local std::vector<JointDescription> pose;pose.resize(jointCount);AnimationError error=AnimationError::None;if(!b.animation->sampleClip(b.animation->self,clip,skeleton,time,looping,pose.data(),jointCount,&error)){resultNil(r,0);*o=1;return true;}
    const auto&v=pose[joint];std::uint32_t k=0;for(float x:v.translation)resultNumber(r,k++,x);for(float x:v.rotation)resultNumber(r,k++,x);for(float x:v.scale)resultNumber(r,k++,x);*o=k;return true;
}

bool buildGlobalTransforms(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    std::uint32_t player=0,skeleton=0,joint=0;if(n!=3||c<16||!uint32Value(a,n,0,player)||!uint32Value(a,n,1,skeleton)||!uint32Value(a,n,2,joint))return false;
    auto&b=*static_cast<Bridge*>(u);std::lock_guard apiLock(b.apiMutex);std::uint32_t jointCount=0;if(!b.animation->skeletonInfo(b.animation->self,skeleton,&jointCount)||joint>=jointCount){resultNil(r,0);*o=1;return true;}
    thread_local std::vector<JointDescription> pose;thread_local std::vector<float> matrices;pose.resize(jointCount);matrices.resize(static_cast<std::size_t>(jointCount)*16);
    AnimationError error=AnimationError::None;if(!b.animation->evaluatePlayer(b.animation->self,player,skeleton,pose.data(),jointCount,&error)
        ||!b.animation->buildGlobalTransforms(b.animation->self,skeleton,pose.data(),matrices.data(),jointCount,&error)){resultNil(r,0);*o=1;return true;}
    const float *matrix=matrices.data()+static_cast<std::size_t>(joint)*16;for(std::uint32_t i=0;i<16;++i)resultNumber(r,i,matrix[i]);*o=16;return true;
}

struct Binding{const char*name;ScriptNativeFunction function;};
constexpr Binding bindings[]={
    {"skeletal_animation.begin_skeleton",beginSkeleton},{"skeletal_animation.set_joint",setJoint},
    {"skeletal_animation.finish_skeleton",finishSkeleton},{"skeletal_animation.begin_clip",beginClip},
    {"skeletal_animation.begin_track",beginTrack},{"skeletal_animation.add_keyframe",addKeyframe},
    {"skeletal_animation.finish_clip",finishClip},{"skeletal_animation.cancel_builder",cancelBuilder},
    {"skeletal_animation.destroy_skeleton",destroySkeleton},{"skeletal_animation.destroy_clip",destroyClip},
    {"skeletal_animation.skeleton_joint_count",skeletonJointCount},{"skeletal_animation.clip_duration",clipDuration},
    {"skeletal_animation.create_player",createPlayer},{"skeletal_animation.destroy_player",destroyPlayer},
    {"skeletal_animation.set_player_clip",setPlayerClip},{"skeletal_animation.set_player_speed",setPlayerSpeed},
    {"skeletal_animation.set_player_playing",setPlayerPlaying},{"skeletal_animation.advance_player",advancePlayer},
    {"skeletal_animation.player_state",playerState},{"skeletal_animation.sample_joint",sampleJoint},
    {"skeletal_animation.evaluate_joint",evaluateJoint},{"skeletal_animation.global_transform",buildGlobalTransforms},
};
}

bool initializeSkeletalAnimationLuaRuntime(void *userData, ScriptRuntimeHandle runtime) noexcept
{
    auto *context=static_cast<SkeletalAnimationLuaBridgeContext*>(userData);
    if(context==nullptr||context->scriptRuntime==nullptr||context->animation==nullptr)return false;
    try
    {
        for (const Binding &binding : bindings)
        {
            if (!context->scriptRuntime->registerNativeFunction(context->scriptRuntime->self,
                    runtime, binding.name, binding.function, context))
                return false;
        }
        return true;
    }
    catch (...)
    {
        return false;
    }
}

void shutdownSkeletalAnimationLuaBridge(SkeletalAnimationLuaBridgeContext &context) noexcept
{
    std::lock_guard lock(context.mutex);
    context.skeletonBuilders.clear();
    context.clipBuilders.clear();
    context.pendingJoints = 0;
    context.pendingKeyframes = 0;
}

} // namespace engine::skeletal_animation
