#include "lua_bridge.hpp"

#include <engine/script_runtime/native_values.hpp>

#include <algorithm>
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
using script_runtime::native::boolean;
using script_runtime::native::floatValue;
using script_runtime::native::int32Value;
using script_runtime::native::resultBoolean;
using script_runtime::native::resultNil;
using script_runtime::native::resultNumber;
using script_runtime::native::resultString;
using script_runtime::native::stringValue;
using script_runtime::native::uint32Value;
using Bridge = SkeletalAnimationLuaBridgeContext;

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

bool addEventMarker(void *userData, const ScriptValue *args, std::uint32_t count,
    ScriptValue *results, std::uint32_t capacity, std::uint32_t *outCount)
{
    std::uint32_t builderHandle = 0;
    float timeSeconds = 0.0f;
    std::uint32_t eventId = 0;
    std::uint32_t userDataValue = 0;
    if (count != 4 || capacity < 1 || !uint32Value(args, count, 0, builderHandle)
        || !floatValue(args, count, 1, timeSeconds)
        || !uint32Value(args, count, 2, eventId)
        || !uint32Value(args, count, 3, userDataValue))
        return false;

    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.mutex);
    auto found = bridge.clipBuilders.find(builderHandle);
    if (found == bridge.clipBuilders.end())
    {
        resultBoolean(results, 0, false);
        *outCount = 1;
        return true;
    }
    if (found->second.eventMarkers.size() >= bridge.maximumEventMarkersPerClip
        || bridge.pendingEventMarkers >= bridge.maximumPendingEventMarkers)
    {
        resultBoolean(results, 0, false);
        *outCount = 1;
        return true;
    }
    try
    {
        found->second.eventMarkers.push_back(AnimationEventMarker{
            .timeSeconds = timeSeconds,
            .eventId = eventId,
            .userData = userDataValue});
        ++bridge.pendingEventMarkers;
        resultBoolean(results, 0, true);
        *outCount = 1;
        return true;
    }
    catch (...)
    {
        resultBoolean(results, 0, false);
        *outCount = 1;
        return true;
    }
}

bool finishClip(void *userData,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*outCount)
{
    auto &b=*static_cast<Bridge*>(userData);std::uint32_t handle=0;if(n!=1||cap<2||!uint32Value(a,n,0,handle))return false;Bridge::ClipBuilder builder;
    {std::lock_guard lock(b.mutex);auto found=b.clipBuilders.find(handle);if(found==b.clipBuilders.end()){resultNil(r,0);resultError(r,1,AnimationError::InvalidHandle);*outCount=2;return true;}
     if(found->second.tracks.empty()||std::any_of(found->second.tracks.begin(),found->second.tracks.end(),[](const auto&t){return t.keyframes.empty();})){resultNil(r,0);resultError(r,1,AnimationError::InvalidDescription);*outCount=2;return true;}
     std::uint64_t keyCount=0;for(const auto&track:found->second.tracks)keyCount+=track.keyframes.size();
     b.pendingKeyframes-=std::min(b.pendingKeyframes,keyCount);
     b.pendingEventMarkers-=std::min<std::uint64_t>(b.pendingEventMarkers,found->second.eventMarkers.size());
     builder=std::move(found->second);b.clipBuilders.erase(found);}
    std::vector<AnimationTrackDescription> tracks;tracks.reserve(builder.tracks.size());for(const auto&track:builder.tracks){tracks.push_back(AnimationTrackDescription{.jointIndex=track.jointIndex,.keyframes=track.keyframes.data(),.keyframeCount=static_cast<std::uint32_t>(track.keyframes.size()),.interpolation=track.interpolation});}
    AnimationClipDescription d{builder.name.c_str(),builder.durationSeconds,tracks.data(),static_cast<std::uint32_t>(tracks.size()),
        builder.eventMarkers.data(),static_cast<std::uint32_t>(builder.eventMarkers.size())};AnimationError error=AnimationError::None;
    std::lock_guard apiLock(b.apiMutex);auto created=b.animation->createClip(b.animation->self,&d,&error);resultHandle(r,0,created);resultError(r,1,error);*outCount=2;return true;
}

bool cancelBuilder(void *userData,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t cap,std::uint32_t*outCount)
{
    auto&b=*static_cast<Bridge*>(userData);std::uint32_t handle=0;if(n!=1||cap<1||!uint32Value(a,n,0,handle))return false;std::lock_guard lock(b.mutex);
    bool ok=false;auto skeleton=b.skeletonBuilders.find(handle);if(skeleton!=b.skeletonBuilders.end()){
        b.pendingJoints-=std::min<std::uint64_t>(b.pendingJoints,skeleton->second.joints.size());b.skeletonBuilders.erase(skeleton);ok=true;
    }else{auto clip=b.clipBuilders.find(handle);if(clip!=b.clipBuilders.end()){
        std::uint64_t count=0;for(const auto&track:clip->second.tracks)count+=track.keyframes.size();
        b.pendingKeyframes-=std::min(b.pendingKeyframes,count);
        b.pendingEventMarkers-=std::min<std::uint64_t>(b.pendingEventMarkers,clip->second.eventMarkers.size());
        b.clipBuilders.erase(clip);ok=true;
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

bool extractRootMotion(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    std::uint32_t clip=0,skeleton=0,root=0;float start=0.0f,end=0.0f;bool looping=false;
    if(n!=6||c<8||!uint32Value(a,n,0,clip)||!uint32Value(a,n,1,skeleton)
        ||!uint32Value(a,n,2,root)||!floatValue(a,n,3,start)||!floatValue(a,n,4,end)
        ||!boolean(a,n,5,looping))return false;
    float translation[3]{},rotation[4]{};AnimationError error=AnimationError::None;
    auto&b=*static_cast<Bridge*>(u);std::lock_guard lock(b.apiMutex);
    const bool ok=b.animation->extractRootMotion(b.animation->self,clip,skeleton,root,start,end,looping,
        translation,rotation,&error);
    if(ok){std::uint32_t k=0;for(float v:translation)resultNumber(r,k++,v);for(float v:rotation)resultNumber(r,k++,v);}
    else for(std::uint32_t k=0;k<7;++k)resultNil(r,k);
    resultError(r,7,error);*o=8;return true;
}

bool playerEventCount(void *userData, const ScriptValue *args, std::uint32_t count,
    ScriptValue *results, std::uint32_t capacity, std::uint32_t *outCount)
{
    std::uint32_t player = 0;
    if (count != 1 || capacity < 2 || !uint32Value(args, count, 0, player))
        return false;
    auto &bridge = *static_cast<Bridge *>(userData);
    std::uint32_t eventCount = 0;
    bool overflowed = false;
    std::lock_guard lock(bridge.apiMutex);
    if (!bridge.animation->playerEventCount(bridge.animation->self, player, &eventCount, &overflowed))
    {
        resultNil(results, 0);
        resultBoolean(results, 1, false);
    }
    else
    {
        resultNumber(results, 0, eventCount);
        resultBoolean(results, 1, overflowed);
    }
    *outCount = 2;
    return true;
}

bool playerEventAt(void *userData, const ScriptValue *args, std::uint32_t count,
    ScriptValue *results, std::uint32_t capacity, std::uint32_t *outCount)
{
    std::uint32_t player = 0;
    std::uint32_t index = 0;
    if (count != 2 || capacity < 5 || !uint32Value(args, count, 0, player)
        || !uint32Value(args, count, 1, index))
        return false;
    auto &bridge = *static_cast<Bridge *>(userData);
    AnimationPlayerEvent event{};
    std::uint32_t copied = 0;
    std::lock_guard lock(bridge.apiMutex);
    if (!bridge.animation->copyPlayerEvents(bridge.animation->self, player, index, &event, 1, &copied)
        || copied == 0)
    {
        resultNil(results, 0);
        *outCount = 1;
        return true;
    }
    resultNumber(results, 0, event.clip);
    resultNumber(results, 1, event.eventId);
    resultNumber(results, 2, event.userData);
    resultNumber(results, 3, event.clipTimeSeconds);
    resultNumber(results, 4, event.direction);
    *outCount = 5;
    return true;
}

bool clearPlayerEvents(void *userData, const ScriptValue *args, std::uint32_t count,
    ScriptValue *results, std::uint32_t capacity, std::uint32_t *outCount)
{
    std::uint32_t player = 0;
    if (count != 1 || capacity < 1 || !uint32Value(args, count, 0, player))
        return false;
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    resultBoolean(results, 0, bridge.animation->clearPlayerEvents(bridge.animation->self, player));
    *outCount = 1;
    return true;
}

bool setPlayerTime(void *userData, const ScriptValue *args, std::uint32_t count,
    ScriptValue *results, std::uint32_t capacity, std::uint32_t *outCount)
{
    std::uint32_t player = 0;
    float timeSeconds = 0.0f;
    if (count != 2 || capacity < 1 || !uint32Value(args, count, 0, player)
        || !floatValue(args, count, 1, timeSeconds))
        return false;
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    resultBoolean(results, 0,
        bridge.animation->setPlayerTime(bridge.animation->self, player, timeSeconds));
    *outCount = 1;
    return true;
}

bool clipInfo(void *userData, const ScriptValue *args, std::uint32_t count,
    ScriptValue *results, std::uint32_t capacity, std::uint32_t *outCount)
{
    std::uint32_t clip = 0;
    if (count != 1 || capacity < 5 || !uint32Value(args, count, 0, clip))
        return false;
    auto &bridge = *static_cast<Bridge *>(userData);
    AnimationClipInfo info{};
    std::lock_guard lock(bridge.apiMutex);
    if (!bridge.animation->clipInfo(bridge.animation->self, clip, &info))
    {
        resultNil(results, 0);
        *outCount = 1;
        return true;
    }
    resultString(results, 0, info.name);
    resultNumber(results, 1, info.durationSeconds);
    resultNumber(results, 2, info.trackCount);
    // uint64 -> double: keyframe counts never approach 2^53 in practice, but
    // explicit conversion keeps -Wconversion clean.
    resultNumber(results, 3, static_cast<double>(info.keyframeCount));
    resultNumber(results, 4, info.eventMarkerCount);
    *outCount = 5;
    return true;
}

bool inverseBindPose(void *userData, const ScriptValue *args, std::uint32_t count,
    ScriptValue *results, std::uint32_t capacity, std::uint32_t *outCount)
{
    std::uint32_t skeleton = 0;
    std::uint32_t joint = 0;
    if (count != 2 || capacity < 16 || !uint32Value(args, count, 0, skeleton)
        || !uint32Value(args, count, 1, joint))
        return false;
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    std::uint32_t jointCount = 0;
    if (!bridge.animation->skeletonInfo(bridge.animation->self, skeleton, &jointCount)
        || joint >= jointCount)
    {
        resultNil(results, 0);
        *outCount = 1;
        return true;
    }
    thread_local std::vector<float> matrices;
    matrices.resize(static_cast<std::size_t>(jointCount) * 16);
    AnimationError error = AnimationError::None;
    if (!bridge.animation->buildInverseBindPose(bridge.animation->self, skeleton,
            matrices.data(), jointCount, &error))
    {
        resultNil(results, 0);
        *outCount = 1;
        return true;
    }
    const float *matrix = matrices.data() + static_cast<std::size_t>(joint) * 16;
    for (std::uint32_t i = 0; i < 16; ++i)
        resultNumber(results, i, matrix[i]);
    *outCount = 16;
    return true;
}

bool skinningMatrix(void *userData, const ScriptValue *args, std::uint32_t count,
    ScriptValue *results, std::uint32_t capacity, std::uint32_t *outCount)
{
    std::uint32_t player = 0;
    std::uint32_t skeleton = 0;
    std::uint32_t joint = 0;
    if (count != 3 || capacity < 16 || !uint32Value(args, count, 0, player)
        || !uint32Value(args, count, 1, skeleton) || !uint32Value(args, count, 2, joint))
        return false;
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    std::uint32_t jointCount = 0;
    if (!bridge.animation->skeletonInfo(bridge.animation->self, skeleton, &jointCount)
        || joint >= jointCount)
    {
        resultNil(results, 0);
        *outCount = 1;
        return true;
    }
    thread_local std::vector<JointDescription> pose;
    thread_local std::vector<float> matrices;
    pose.resize(jointCount);
    matrices.resize(static_cast<std::size_t>(jointCount) * 16);
    AnimationError error = AnimationError::None;
    if (!bridge.animation->evaluatePlayer(bridge.animation->self, player, skeleton,
            pose.data(), jointCount, &error)
        || !bridge.animation->computeSkinningMatrices(bridge.animation->self, skeleton,
            pose.data(), matrices.data(), jointCount, &error))
    {
        resultNil(results, 0);
        *outCount = 1;
        return true;
    }
    const float *matrix = matrices.data() + static_cast<std::size_t>(joint) * 16;
    for (std::uint32_t i = 0; i < 16; ++i)
        resultNumber(results, i, matrix[i]);
    *outCount = 16;
    return true;
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
    {"skeletal_animation.extract_root_motion",extractRootMotion},
    {"skeletal_animation.add_event_marker",addEventMarker},
    {"skeletal_animation.player_event_count",playerEventCount},
    {"skeletal_animation.player_event_at",playerEventAt},
    {"skeletal_animation.clear_player_events",clearPlayerEvents},
    {"skeletal_animation.evaluate_joint",evaluateJoint},{"skeletal_animation.global_transform",buildGlobalTransforms},
    {"skeletal_animation.set_player_time",setPlayerTime},
    {"skeletal_animation.clip_info",clipInfo},
    {"skeletal_animation.inverse_bind_pose",inverseBindPose},
    {"skeletal_animation.skinning_matrix",skinningMatrix},
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
    context.pendingEventMarkers = 0;
}

} // namespace engine::skeletal_animation
