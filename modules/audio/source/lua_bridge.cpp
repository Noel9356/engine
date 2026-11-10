#include "lua_bridge.hpp"

#include <engine/script_runtime/native_values.hpp>

#include <cstdint>
#include <limits>

namespace engine::audio
{
namespace
{
using script_runtime::ScriptNativeFunction;
using script_runtime::ScriptRuntimeHandle;
using script_runtime::ScriptValue;
using script_runtime::ScriptValueType;
using script_runtime::native::floatValue;
using script_runtime::native::number;
using script_runtime::native::optionalBoolean;
using script_runtime::native::resultBoolean;
using script_runtime::native::resultNil;
using script_runtime::native::resultNumber;
using script_runtime::native::resultString;
using script_runtime::native::stringValue;
using script_runtime::native::uint32Value;
AudioCapability &audio(void *userData)
{
    return *static_cast<AudioLuaBridgeContext *>(userData)->audio;
}

void resultError(ScriptValue *results, std::uint32_t index, AudioError error)
{
    resultNumber(results, index, static_cast<double>(static_cast<std::uint32_t>(error)));
}

bool parseEngineSound(const ScriptValue *args, std::uint32_t count, AudioEngineHandle &engine, AudioSoundHandle &sound)
{
    std::uint32_t engineValue = 0;
    std::uint32_t soundValue = 0;
    if (!uint32Value(args, count, 0, engineValue) || !uint32Value(args, count, 1, soundValue))
        return false;
    engine = engineValue;
    sound = soundValue;
    return true;
}

bool createEngine(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    if (count > 6 || capacity < 2) return false;
    auto &api = audio(userData);
    AudioEngineDescription d{};
    float master = -1.0f;
    if (!uint32Value(args, count, 0, d.sampleRate, 0, false)
        || !uint32Value(args, count, 1, d.channels, 0, false)
        || !uint32Value(args, count, 2, d.listenerCount, 0, false)
        || !floatValue(args, count, 3, master, -1.0f, false)
        || !optionalBoolean(args, count, 4, d.noDevice)
        || !optionalBoolean(args, count, 5, d.noAutoStart)) return false;
    d.masterVolume = master;
    AudioError error = AudioError::None;
    const auto handle = api.createEngine(api.self, &d, &error);
    if (handle == invalidAudioEngine) resultNil(out, 0); else resultNumber(out, 0, handle);
    resultError(out, 1, error);
    *outCount = 2;
    return true;
}

bool destroyEngine(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *,
    std::uint32_t, std::uint32_t *outCount)
{
    std::uint32_t h = 0; if (count != 1 || !uint32Value(args,count,0,h)) return false;
    audio(userData).destroyEngine(audio(userData).self,h);
    *outCount=0; return true;
}

bool engineInfo(void *userData,const ScriptValue *args,std::uint32_t count,ScriptValue*out,std::uint32_t cap,std::uint32_t*outCount)
{
    std::uint32_t h=0; if(count!=1||cap<4||!uint32Value(args,count,0,h))return false;
    AudioEngineInfo info{}; auto &api=audio(userData);
    if(!api.engineInfo(api.self,h,&info)){resultNil(out,0);*outCount=1;return true;}
    resultNumber(out,0,info.sampleRate); resultNumber(out,1,info.channels); resultBoolean(out,2,info.hasDevice);
    resultString(out,3,info.backendName); *outCount=4; return true;
}

bool engineAction(void *userData,const ScriptValue*args,std::uint32_t count,ScriptValue*out,std::uint32_t cap,std::uint32_t*outCount,bool start)
{
    std::uint32_t h=0; if(count!=1||cap<2||!uint32Value(args,count,0,h))return false;
    auto&api=audio(userData); AudioError error=AudioError::None;
    const bool ok=start?api.startEngine(api.self,h,&error):api.stopEngine(api.self,h,&error);
    resultBoolean(out,0,ok); resultError(out,1,error); *outCount=2; return true;
}
bool startEngine(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return engineAction(u,a,n,r,c,o,true);}
bool stopEngine(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return engineAction(u,a,n,r,c,o,false);}

bool setMasterVolume(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    std::uint32_t h=0; float v=0; if(n!=2||c<1||!uint32Value(a,n,0,h)||!floatValue(a,n,1,v))return false;
    resultBoolean(r,0,audio(u).setMasterVolume(audio(u).self,h,v));*o=1;return true;
}

bool createBus(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    if (n > 4 || c < 2)
        return false;
    std::uint32_t e = 0;
    AudioBusDescription d{};
    std::uint32_t parent = 0;
    if(!uint32Value(a,n,0,e)||!uint32Value(a,n,1,parent,0,false)||!floatValue(a,n,2,d.volume,1.0f,false)||!optionalBoolean(a,n,3,d.muted))return false;
    d.parent=parent; AudioError err=AudioError::None; auto&api=audio(u);
    auto h=api.createBus(api.self,e,&d,&err); if(h==invalidAudioBus)resultNil(r,0);else resultNumber(r,0,h);resultError(r,1,err);*o=2;return true;
}
bool destroyBus(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    std::uint32_t e=0,b=0;if(n!=2||c<2||!uint32Value(a,n,0,e)||!uint32Value(a,n,1,b))return false;
    AudioError err=AudioError::None;auto&api=audio(u);bool ok=api.destroyBus(api.self,e,b,&err);
    resultBoolean(r,0,ok);resultError(r,1,err);*o=2;return true;
}
bool setBusVolume(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    std::uint32_t e=0,b=0;float v=0;if(n!=3||c<1||!uint32Value(a,n,0,e)||!uint32Value(a,n,1,b)||!floatValue(a,n,2,v))return false;
    auto&api=audio(u);resultBoolean(r,0,api.setBusVolume(api.self,e,b,v));*o=1;return true;
}
bool setBusMuted(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    std::uint32_t e=0,b=0;bool v=false;if(n!=3||c<1||!uint32Value(a,n,0,e)||!uint32Value(a,n,1,b)||!optionalBoolean(a,n,2,v,false,true))return false;
    auto&api=audio(u);resultBoolean(r,0,api.setBusMuted(api.self,e,b,v));*o=1;return true;
}

bool createSound(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    if (n < 2 || n > 12 || c < 2)
        return false;
    std::uint32_t e = 0;
    std::uint32_t b = 0;
    const char *path = nullptr;
    AudioSoundDescription d{};
    if(!uint32Value(a,n,0,e)||!stringValue(a,n,1,path)||!uint32Value(a,n,2,b,0,false)
       ||!floatValue(a,n,3,d.volume,1.0f,false)||!floatValue(a,n,4,d.pitch,1.0f,false)||!floatValue(a,n,5,d.pan,0.0f,false)
       ||!optionalBoolean(a,n,6,d.looping)||!optionalBoolean(a,n,7,d.spatialized)||!optionalBoolean(a,n,8,d.streaming)
       ||!optionalBoolean(a,n,9,d.decode)||!optionalBoolean(a,n,10,d.asynchronous)||!optionalBoolean(a,n,11,d.disablePitch))return false;
    d.filePath=path;d.bus=b;AudioError err=AudioError::None;auto&api=audio(u);
    auto h=api.createSound(api.self,e,&d,&err);if(h==invalidAudioSound)resultNil(r,0);else resultNumber(r,0,h);resultError(r,1,err);*o=2;return true;
}
bool destroySound(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*,std::uint32_t,std::uint32_t*o)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;if(n!=2||!parseEngineSound(a,n,e,s))return false;
    auto&api=audio(u);api.destroySound(api.self,e,s);*o=0;return true;
}
enum class SoundAction{Play,Pause,Stop};
bool soundAction(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o,SoundAction action)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;if(n!=2||c<2||!parseEngineSound(a,n,e,s))return false;AudioError err=AudioError::None;auto&api=audio(u);
    bool ok=action==SoundAction::Play?api.play(api.self,e,s,&err):(action==SoundAction::Pause?api.pause(api.self,e,s,&err):api.stop(api.self,e,s,&err));
    resultBoolean(r,0,ok);resultError(r,1,err);*o=2;return true;
}
bool play(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return soundAction(u,a,n,r,c,o,SoundAction::Play);}
bool pause(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return soundAction(u,a,n,r,c,o,SoundAction::Pause);}
bool stop(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return soundAction(u,a,n,r,c,o,SoundAction::Stop);}

bool seekSeconds(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;double t=0;if(n!=3||c<1||!parseEngineSound(a,n,e,s)||!number(a,n,2,t))return false;
    auto&api=audio(u);resultBoolean(r,0,api.seekSeconds(api.self,e,s,t));*o=1;return true;
}
bool querySound(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o,bool end)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;if(n!=2||c<1||!parseEngineSound(a,n,e,s))return false;auto&api=audio(u);
    resultBoolean(r,0,end?api.atEnd(api.self,e,s):api.isPlaying(api.self,e,s));*o=1;return true;
}
bool isPlaying(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return querySound(u,a,n,r,c,o,false);}
bool atEnd(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return querySound(u,a,n,r,c,o,true);}
bool soundTimes(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;if(n!=2||c<2||!parseEngineSound(a,n,e,s))return false;double cursor=0,length=0;auto&api=audio(u);
    if(!api.soundTimes(api.self,e,s,&cursor,&length)){resultNil(r,0);resultNil(r,1);}else{resultNumber(r,0,cursor);resultNumber(r,1,length);}*o=2;return true;
}

enum class FloatSetter{Volume,Pitch,Pan,Doppler};
bool floatSoundSetter(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o,FloatSetter which)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;float v=0;if(n!=3||c<1||!parseEngineSound(a,n,e,s)||!floatValue(a,n,2,v))return false;auto&api=audio(u);bool ok=false;
    switch(which){case FloatSetter::Volume:ok=api.setSoundVolume(api.self,e,s,v);break;case FloatSetter::Pitch:ok=api.setSoundPitch(api.self,e,s,v);break;case FloatSetter::Pan:ok=api.setSoundPan(api.self,e,s,v);break;case FloatSetter::Doppler:{AudioError err=AudioError::None;ok=api.setSoundDopplerFactor(api.self,e,s,v,&err);break;}}
    resultBoolean(r,0,ok);*o=1;return true;
}
bool setSoundVolume(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return floatSoundSetter(u,a,n,r,c,o,FloatSetter::Volume);}
bool setSoundPitch(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return floatSoundSetter(u,a,n,r,c,o,FloatSetter::Pitch);}
bool setSoundPan(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return floatSoundSetter(u,a,n,r,c,o,FloatSetter::Pan);}
bool setSoundDoppler(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return floatSoundSetter(u,a,n,r,c,o,FloatSetter::Doppler);}

bool setSoundLooping(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;bool b=false;if(n!=3||c<1||!parseEngineSound(a,n,e,s)||!optionalBoolean(a,n,2,b,false,true))return false;
    resultBoolean(r,0,audio(u).setSoundLooping(audio(u).self,e,s,b));*o=1;return true;
}
enum class VectorSetter{Position,Velocity,Direction};
bool vectorSoundSetter(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o,VectorSetter which)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;float v[3]{};if(n!=5||c<1||!parseEngineSound(a,n,e,s)||!floatValue(a,n,2,v[0])||!floatValue(a,n,3,v[1])||!floatValue(a,n,4,v[2]))return false;
    auto&api=audio(u);bool ok=false;switch(which){case VectorSetter::Position:ok=api.setSoundPosition(api.self,e,s,v);break;case VectorSetter::Velocity:ok=api.setSoundVelocity(api.self,e,s,v);break;case VectorSetter::Direction:{AudioError err=AudioError::None;ok=api.setSoundDirection(api.self,e,s,v,&err);break;}}
    resultBoolean(r,0,ok);*o=1;return true;
}
bool setSoundPosition(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return vectorSoundSetter(u,a,n,r,c,o,VectorSetter::Position);}
bool setSoundVelocity(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return vectorSoundSetter(u,a,n,r,c,o,VectorSetter::Velocity);}
bool setSoundDirection(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o){return vectorSoundSetter(u,a,n,r,c,o,VectorSetter::Direction);}

bool setSoundCone(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;float inner=0,outer=0,gain=0;if(n!=5||c<1||!parseEngineSound(a,n,e,s)||!floatValue(a,n,2,inner)||!floatValue(a,n,3,outer)||!floatValue(a,n,4,gain))return false;
    AudioError err=AudioError::None;auto&api=audio(u);bool ok=api.setSoundCone(api.self,e,s,inner,outer,gain,&err);resultBoolean(r,0,ok);*o=1;return true;
}
bool setSoundListener(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;std::uint32_t listener=0;if(n!=3||c<1||!parseEngineSound(a,n,e,s)||!uint32Value(a,n,2,listener,std::numeric_limits<std::uint32_t>::max()))return false;
    AudioError err=AudioError::None;auto&api=audio(u);bool ok=api.setSoundListener(api.self,e,s,listener,&err);resultBoolean(r,0,ok);*o=1;return true;
}
bool setSoundSpatialization(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;bool enabled=false;if(n!=3||c<1||!parseEngineSound(a,n,e,s)||!optionalBoolean(a,n,2,enabled,false,true))return false;
    AudioError err=AudioError::None;auto&api=audio(u);bool ok=api.setSoundSpatializationEnabled(api.self,e,s,enabled,&err);resultBoolean(r,0,ok);*o=1;return true;
}
bool setSoundGainRange(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;float low=0,high=0;if(n!=4||c<1||!parseEngineSound(a,n,e,s)||!floatValue(a,n,2,low)||!floatValue(a,n,3,high))return false;
    AudioError err=AudioError::None;auto&api=audio(u);bool ok=api.setSoundGainRange(api.self,e,s,low,high,&err);resultBoolean(r,0,ok);*o=1;return true;
}
bool setSoundAttenuation(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;float minD=0,maxD=0,roll=0;std::uint32_t model=0;
    if(n!=6||c<1||!parseEngineSound(a,n,e,s)||!floatValue(a,n,2,minD)||!floatValue(a,n,3,maxD)||!floatValue(a,n,4,roll)||!uint32Value(a,n,5,model))return false;
    if(model>static_cast<std::uint32_t>(AudioAttenuationModel::Exponential))return false;
    AudioError err=AudioError::None;auto&api=audio(u);bool ok=api.setSoundAttenuation(api.self,e,s,minD,maxD,roll,static_cast<AudioAttenuationModel>(model),&err);resultBoolean(r,0,ok);*o=1;return true;
}
bool fadeSoundGain(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;float gain=0;double seconds=0;if(n!=4||c<2||!parseEngineSound(a,n,e,s)||!floatValue(a,n,2,gain)||!number(a,n,3,seconds))return false;
    AudioError err=AudioError::None;auto&api=audio(u);bool ok=api.fadeSoundGain(api.self,e,s,gain,seconds,&err);resultBoolean(r,0,ok);resultError(r,1,err);*o=2;return true;
}
bool schedulePlay(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;double delay=0;if(n!=3||c<2||!parseEngineSound(a,n,e,s)||!number(a,n,2,delay))return false;
    AudioError err=AudioError::None;auto&api=audio(u);bool ok=api.schedulePlay(api.self,e,s,delay,&err);resultBoolean(r,0,ok);resultError(r,1,err);*o=2;return true;
}
bool scheduleStop(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    AudioEngineHandle e=0;AudioSoundHandle s=0;double delay=0,fade=0;if(n!=4||c<2||!parseEngineSound(a,n,e,s)||!number(a,n,2,delay)||!number(a,n,3,fade))return false;
    AudioError err=AudioError::None;auto&api=audio(u);bool ok=api.scheduleStop(api.self,e,s,delay,fade,&err);resultBoolean(r,0,ok);resultError(r,1,err);*o=2;return true;
}
bool setListener(void*u,const ScriptValue*a,std::uint32_t n,ScriptValue*r,std::uint32_t c,std::uint32_t*o)
{
    std::uint32_t e=0,index=0;AudioListenerDescription d{};
    if(n!=14||c<1||!uint32Value(a,n,0,e)||!uint32Value(a,n,1,index))return false;
    std::uint32_t k=2;for(float &v:d.position){if(!floatValue(a,n,k++,v))return false;}for(float &v:d.direction){if(!floatValue(a,n,k++,v))return false;}
    for(float &v:d.worldUp){if(!floatValue(a,n,k++,v))return false;}for(float &v:d.velocity){if(!floatValue(a,n,k++,v))return false;}
    auto&api=audio(u);resultBoolean(r,0,api.setListener(api.self,e,index,&d));*o=1;return true;
}

struct Binding { const char *name; ScriptNativeFunction function; };
constexpr Binding bindings[] = {
    {"audio.create_engine", createEngine}, {"audio.destroy_engine", destroyEngine}, {"audio.engine_info", engineInfo},
    {"audio.start_engine", startEngine}, {"audio.stop_engine", stopEngine}, {"audio.set_master_volume", setMasterVolume},
    {"audio.create_bus", createBus}, {"audio.destroy_bus", destroyBus}, {"audio.set_bus_volume", setBusVolume},
    {"audio.set_bus_muted", setBusMuted}, {"audio.create_sound", createSound}, {"audio.destroy_sound", destroySound},
    {"audio.play", play}, {"audio.pause", pause}, {"audio.stop", stop}, {"audio.seek_seconds", seekSeconds},
    {"audio.is_playing", isPlaying}, {"audio.at_end", atEnd}, {"audio.sound_times", soundTimes},
    {"audio.set_sound_volume", setSoundVolume}, {"audio.set_sound_pitch", setSoundPitch}, {"audio.set_sound_pan", setSoundPan},
    {"audio.set_sound_looping", setSoundLooping}, {"audio.set_sound_position", setSoundPosition},
    {"audio.set_sound_velocity", setSoundVelocity}, {"audio.set_sound_direction", setSoundDirection},
    {"audio.set_sound_cone", setSoundCone}, {"audio.set_sound_doppler_factor", setSoundDoppler},
    {"audio.set_sound_listener", setSoundListener}, {"audio.set_sound_spatialization_enabled", setSoundSpatialization},
    {"audio.set_sound_gain_range", setSoundGainRange}, {"audio.set_sound_attenuation", setSoundAttenuation},
    {"audio.fade_sound_gain", fadeSoundGain}, {"audio.schedule_play", schedulePlay}, {"audio.schedule_stop", scheduleStop},
    {"audio.set_listener", setListener},
};
}

bool initializeAudioLuaRuntime(void *userData, ScriptRuntimeHandle runtime) noexcept
{
    auto *context = static_cast<AudioLuaBridgeContext *>(userData);
    if (context == nullptr || context->scriptRuntime == nullptr || context->audio == nullptr)
        return false;

    // Every closure retains this same context pointer so script_runtime can
    // disable it safely when the module unregisters the initializer.
    try
    {
        for (const Binding &binding : bindings)
        {
            if (!context->scriptRuntime->registerNativeFunction(context->scriptRuntime->self, runtime,
                    binding.name, binding.function, context))
                return false;
        }
        return true;
    }
    catch (...)
    {
        return false;
    }
}

} // namespace engine::audio
