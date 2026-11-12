-- character_lab.lua - exercises the character module's Lua bridge.
--
-- Activated by the character-customisation demo by default (or with
-- --active-plugin character_lab elsewhere). On start it builds a small
-- assembly from the demo content through the character.* natives, edits it,
-- and reports the results; on shutdown the assembly is destroyed.

-- The demo registers its body part content when the character screen is
-- built (before the first frame), so assembly creation happens lazily on the
-- first frame rather than in on_start.
local assembly   = nil
local lab_frames = 0

local function report(message)
    print("character_lab: " .. message)
end

function on_construct()
    report("loaded (Lua bridge to the character module)")
end

function on_start()
    report("waiting for the demo content registry (first frame)")
end

function on_frame(delta)
    if assembly ~= nil then
        return
    end
    lab_frames = lab_frames + 1
    if lab_frames < 2 then
        return
    end
    assembly = character.create("hybrid_biped")
    if assembly == 0 then
        report("could not create the hybrid_biped assembly")
        return
    end
    local parts = character.instance_count(assembly)
    report(string.format("assembly %d created with %d parts", assembly, parts))

    -- Parameters: central registry, per-asset bindings.
    character.set_parameter(assembly, "body.height", 1.2)
    character.set_parameter(assembly, "tail.length", 1.4)
    local height, ok = character.get_parameter(assembly, "body.height")
    report(string.format("body.height=%.2f (applied=%s)", height, tostring(ok == true)))

    -- Evaluate + validate through the bridge.
    local evaluated = character.evaluate(assembly)
    local issues = character.validate(assembly)
    report(string.format("evaluate=%s issues=%d", tostring(evaluated == true), issues))

    -- Instance enumeration through the bridge.
    for index = 1, character.instance_count(assembly) do
        local id = character.instance_id(assembly, index)
        local name, part, parent = character.instance_info(assembly, id)
        local m = {character.instance_transform(assembly, id)}
        if name == "head" and #m == 16 then
            report(string.format("head instance %d part=%s parent=%d world_y=%.3f",
                id, tostring(part), parent or 0, m[14]))
        end
    end

    -- A second character built from scratch on the same registry.
    local custom = character.create(nil)
    local torso_part = character.add_part(custom, "torso_humanoid", "torso")
    local head_part = character.add_part(custom, "head_dragon", "head")
    local connected = character.connect(custom, torso_part, "neck.base", head_part, "head.root")
    character.evaluate(custom)
    report(string.format("custom assembly %d: torso=%d head=%d connected=%s",
        custom, torso_part, head_part, tostring(connected == true)))
    character.destroy(custom)

    report("ready")
end


function on_shutdown()
    if assembly ~= nil and assembly ~= 0 then
        character.destroy(assembly)
        assembly = nil
        report("assembly destroyed")
    end
end
